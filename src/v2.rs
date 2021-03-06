use crate::{to_ptr, InterruptCfg};
use core::ptr::{read_volatile, write_volatile};

/*******************************************************************************
 * GICv2 miscellaneous definitions
 ******************************************************************************/

// Interrupt IDs reported by the HPPIR and IAR registers
const PENDING_G1_INTID: u32 = 1022;

/*******************************************************************************
 * GICv2 specific Distributor interface register offsets and constants.
 ******************************************************************************/
const GICD_ITARGETSR: usize = 0x800;
const _GICD_SGIR: usize = 0xF00;
const _GICD_CPENDSGIR: usize = 0xF10;
const _GICD_SPENDSGIR: usize = 0xF20;
const GICD_PIDR2_GICV2: usize = 0xFE8;

const _ITARGETSR_SHIFT: u32 = 2;
const _GIC_TARGET_CPU_MASK: u32 = 0xff;

/*******************************************************************************
 * GICv2 specific CPU interface register offsets and constants.
 ******************************************************************************/
// Physical CPU Interface registers
const GICC_CTLR: usize = 0x0; // CPU Interface Control Register
const GICC_PMR: usize = 0x4; // Interrupt Priority Mask Register
const _GICC_BPR: usize = 0x8; // Binary Point Register,
const GICC_IAR: usize = 0xC; // Interrupt Acknowledge Register
const GICC_EOIR: usize = 0x10; // End of Interrupt Register
const GICC_RPR: usize = 0x14; // Running Priority Register
const GICC_HPPIR: usize = 0x18; // Highest Priority Pending Interrupt Register
const GICC_AHPPIR: usize = 0x28; // Aliased Highest Priority Pending Interrupt Register
const _GICC_IIDR: usize = 0xFC; // CPU Interface Identification Register
const _GICC_DIR: usize = 0x1000; // Deactivate Interrupt Register
const _GICC_PRIODROP: usize = GICC_EOIR;

/// Interrupt ID mask for HPPIR, AHPPIR, IAR and AIAR CPU Interface registers
const INT_ID_MASK: u32 = 0x3ff;

bitflags! {
    /// GICC_CTLR register for secure mode
    struct GiccCtlrS: u32 {
        const ENABLE_GRP0      = 0b0000_0000_0001; // Enable Group 0 interrupts.
        const ENABLE_GRP1      = 0b0000_0000_0010; // Enable Group 1 interrupts.
        const ACK_CTL          = 0b0000_0000_0100; // Ack Control.
        const FIQ_EN           = 0b0000_0000_1000; // FIQ Enable.
        const CBPR             = 0b0000_0001_0000; // Control GICC_BPR.
        const FIQ_BYP_DIS_GRP0 = 0b0000_0010_0000; // FIQ Bypass Disable Group 0 interrupts.
        const IRQ_BYP_DIS_GRP0 = 0b0000_0100_0000; // IRQ Bypass Disable Group 0 interrupts.
        const FIQ_BYP_DIS_GRP1 = 0b0000_1000_0000; // FIQ Bypass Disable Group 1 interrupts.
        const IRQ_BYP_DIS_GRP1 = 0b0001_0000_0000; // IRQ Bypass Disable Group 1 interrupts.
        const EOI_MODE_S       = 0b0010_0000_0000; // Control of GICC_EOIR and GICC_DIR.
        const EOI_MODE_NS      = 0b0100_0000_0000; // Alias of EOImodeNS from the Non-secure copy
    }
}

bitflags! {
    /// GICD_CTLR register for secure mode
    struct GicdCtlrS: u32 {
        const ENABLE_GRP0 = 0b01; // Enable Group 0 interrupts.
        const ENABLE_GRP1 = 0b10; // Enable Group 1 interrupts.
    }
}

bitflags! {
    /// Interrupt Target
    pub struct Target: u8 {
        const CPU0 = 0b0000_0001;
        const CPU1 = 0b0000_0010;
        const CPU2 = 0b0000_0100;
        const CPU3 = 0b0000_1000;
        const CPU4 = 0b0001_0000;
        const CPU5 = 0b0010_0000;
        const CPU6 = 0b0100_0000;
        const CPU7 = 0b1000_0000;
        const ALL  = 0xFF;
    }
}

#[inline(always)]
fn dsbishsy() {
    unsafe { asm!("dsb sy") };
}

#[inline(always)]
fn dsbishst() {
    unsafe { asm!("dsb ishst") };
}

#[inline(always)]
fn dmbishst() {
    unsafe { asm!("dmb ishst") };
}

pub struct GICv2 {
    gicd_base: usize,
    gicc_base: usize,
}

impl GICv2 {
    pub fn new(gicd_base: usize, gicc_base: usize) -> GICv2 {
        GICv2 {
            gicd_base,
            gicc_base,
        }
    }

    /// Enable secure interrupts and use FIQs to route them. Disable legacy bypass
    /// and set the priority mask register to allow all interrupts to trickle in.
    pub fn cpuif_enable(&self) {
        let ctlr = GiccCtlrS::ENABLE_GRP0
            | GiccCtlrS::FIQ_EN
            | GiccCtlrS::FIQ_BYP_DIS_GRP0
            | GiccCtlrS::IRQ_BYP_DIS_GRP0
            | GiccCtlrS::FIQ_BYP_DIS_GRP1
            | GiccCtlrS::IRQ_BYP_DIS_GRP1;

        self.gicc_write_pmr(crate::GIC_PRI_MASK as u8);
        self.gicc_write_ctlr(ctlr);
    }

    /// Place the cpu interface in a state where it can never make a cpu exit wfi as
    /// as result of an asserted interrupt. This is critical for powering down a cpu
    pub fn cpuif_disable(&self) {
        // Disable secure, non-secure interrupts and disable their bypass
        let mut ctlr = self.gicc_read_ctlr();
        ctlr |= GiccCtlrS::FIQ_BYP_DIS_GRP1
            | GiccCtlrS::FIQ_BYP_DIS_GRP0
            | GiccCtlrS::IRQ_BYP_DIS_GRP1
            | GiccCtlrS::IRQ_BYP_DIS_GRP0;
        ctlr &= !(GiccCtlrS::ENABLE_GRP0 | GiccCtlrS::ENABLE_GRP1);
        self.gicc_write_ctlr(ctlr);
    }

    /// Per CPU initializeion
    ///
    /// - Setup the default PPI/SGI priorities doing four
    /// - Configure PPIs and SGIs specified by props whose group must be 0. SPIs are ignored.
    pub fn pcpu_distif_init(&self, props: &[crate::InterruptProp]) {
        self.check_v2();
        self.secure_ppi_sgi_setup_props(props);

        // Enable G0 interrupts if not already
        let ctlr = self.gicd_read_ctlr();
        if !ctlr.contains(GicdCtlrS::ENABLE_GRP0) {
            self.gicd_write_ctlr(ctlr | GicdCtlrS::ENABLE_GRP0);
        }
    }

    /// Primary CPU Initialization
    ///
    /// - Treat all SPIs as G1NS by default
    /// - Setup the default SPI priorities doing four
    /// - Treat all SPIs as level triggered by default
    /// - Configure SPIs specified by props whose group must be 0. SGIs and PPIs are ignored.
    pub fn distif_init(&self, props: &[crate::InterruptProp]) {
        self.check_v2();

        // Disable the distributor before going further
        let ctlr = self.gicd_read_ctlr();
        self.gicd_write_ctlr(ctlr & !(GicdCtlrS::ENABLE_GRP0 | GicdCtlrS::ENABLE_GRP1));

        // Set the default attribute of all SPIs
        self.spis_configure_defaults();

        self.secure_spis_configure_props(props);

        // Re-enable the secure SPIs now that they have been configured
        self.gicd_write_ctlr(ctlr | GicdCtlrS::ENABLE_GRP0);
    }

    /// This function returns whether FIQ is enabled in the GIC CPU interface.
    pub fn is_fiq_enabled(&self) -> bool {
        let gicc_ctlr = self.gicc_read_ctlr();
        gicc_ctlr.contains(GiccCtlrS::FIQ_EN)
    }

    /// This function returns the type of the highest priority pending interrupt at
    /// the GIC cpu interface. The return values can be one of the following :
    ///   PENDING_G1_INTID   : The interrupt type is non secure Group 1.
    ///   0 - 1019           : The interrupt type is secure Group 0.
    ///   GIC_SPURIOUS_INTERRUPT : there is no pending interrupt with
    ///                            sufficient priority to be signaled
    pub fn get_pending_interrupt_type(&self) -> u32 {
        self.gicc_read_hppir() & INT_ID_MASK
    }

    /// This function returns the id of the highest priority pending interrupt at
    /// the GIC cpu interface. GIC_SPURIOUS_INTERRUPT is returned when there is no
    /// interrupt pending.
    pub fn get_pending_interrupt_id(&self) -> u32 {
        let id = self.gicc_read_hppir() & INT_ID_MASK;

        // Find out which non-secure interrupt it is under the assumption that
        // the GICC_CTLR.AckCtl bit is 0.
        if id == PENDING_G1_INTID {
            self.gicc_read_ahppir() & INT_ID_MASK
        } else {
            id
        }
    }

    /// This functions reads the GIC cpu interface Interrupt Acknowledge register
    /// to start handling the pending secure 0 interrupt. It returns the
    /// contents of the IAR.
    pub fn acknowledge_interrupt(&self) -> u32 {
        self.gicc_read_iar()
    }

    /// This functions writes the GIC cpu interface End Of Interrupt register with
    /// the passed value to finish handling the active secure group 0 interrupt.
    pub fn end_of_interrupt(&self, id: u32) {
        // Ensure the write to peripheral registers are *complete* before the write
        // to GIC_EOIR.
        //
        // Note: The completion guarantee depends on various factors of system design
        // and the barrier is the best core can do by which execution of further
        // instructions waits till the barrier is alive.
        dsbishsy();
        self.gicc_write_eoir(id);
    }

    /// This function returns the type of the interrupt id depending upon the group
    /// this interrupt has been configured under by the interrupt controller i.e.
    /// group0 secure or group1 non secure. It returns zero for Group 0 secure and
    /// one for Group 1 non secure interrupt.
    pub fn get_interrupt_group(&self, id: u32) -> crate::InterruptGrp {
        self.gicd_get_igroupr(id)
    }

    /// This function returns the priority of the interrupt the processor is
    /// currently servicing.
    pub fn get_running_pirority(&self) -> u32 {
        self.gicc_read_rpr()
    }

    /// This function sets the GICv2 target mask pattern for the current PE. The PE
    /// target mask is used to translate linear PE index (returned by platform core
    /// position) to a bit mask used when targeting interrupts to a PE (for example
    /// when raising SGIs and routing SPIs).
    pub fn set_pe_target_mask(&self) {
        todo!();
    }

    /// This function returns the active status of the interrupt (either because the
    /// state is active, or active and pending).
    pub fn get_interrupt_active(&self, id: u32) -> bool {
        assert!(id <= crate::MAX_SPI_ID);
        self.gicd_get_isactiver(id)
    }

    /// This function enables the interrupt identified by id.
    pub fn enable_interrupt(&self, id: u32) {
        assert!(id <= crate::MAX_SPI_ID);

        // Ensure that any shared variable updates depending on out of band
        // interrupt trigger are observed before enabling interrupt.
        dsbishst();
        self.gicd_set_isenabler(id);
    }

    /// This function disables the interrupt identified by id.
    pub fn disable_interrupt(&self, id: u32) {
        assert!(id <= crate::MAX_SPI_ID);

        // Disable interrupt, and ensure that any shared variable updates
        // depending on out of band interrupt trigger are observed afterwards.
        self.gicd_set_icenabler(id);
        dsbishst();
    }

    /// This function sets the interrupt priority as supplied for the given interrupt
    /// id.
    pub fn set_interrupt_priority(&self, id: u32, priority: u8) {
        assert!(id <= crate::MAX_SPI_ID);
        self.gicd_set_ipriorityr(id, priority);
    }

    /// This function assigns group for the interrupt identified by id. The group can
    /// be any of GICV2_INTR_GROUP*
    pub fn set_interrupt_type(&self, id: u32, int_type: crate::InterruptGrp) {
        assert!(id <= crate::MAX_SPI_ID);

        let _lock = crate::lock();
        match int_type {
            crate::InterruptGrp::Group0 => {
                self.gicd_clr_igroupr(id);
            }
            crate::InterruptGrp::Group1 => {
                self.gicd_set_igroupr(id);
            }
        }
    }

    /// This function raises the specified SGI to requested targets.
    ///
    /// The proc_num parameter must be the linear index of the target PE in the
    /// system.
    pub fn rasise_sgi(&self, _sgi_num: usize, _proc_num: usize) {
        todo!();
    }

    /// This function sets the interrupt routing for the given SPI interrupt id.
    /// The interrupt routing is specified in routing mode. The proc_num parameter is
    /// linear index of the PE to target SPI.
    pub fn set_spi_routing(&self, id: u32, target: Target) {
        assert!(crate::MIN_SPI_ID <= id && id <= crate::MAX_SPI_ID);
        self.gicd_set_itargetsr(id, target);
    }

    /// This function clears the pending status of an interrupt identified by id.
    pub fn clear_interrupt_pending(&self, id: u32) {
        // SGIs can't be cleared pending
        assert!(id >= crate::MIN_PPI_ID);

        // Clear pending interrupt, and ensure that any shared variable updates
        // depending on out of band interrupt trigger are observed afterwards.
        self.gicd_set_icpendr(id);
        dsbishst();
    }

    /// This function sets the pending status of an interrupt identified by id.
    pub fn set_interrupt_pending(&self, id: u32) {
        // SGIs can't be cleared pending
        assert!(id >= crate::MIN_PPI_ID);

        // Ensure that any shared variable updates depending on out of band
        // interrupt trigger are observed before setting interrupt pending.
        dsbishst();
        self.gicd_set_ispendr(id);
    }

    /// This function sets the PMR register with the supplied value. Returns the
    /// original PMR.
    pub fn set_pmr(&self, mask: u8) -> u32 {
        let old_mask = self.gicc_read_pmr();

        // Order memory updates w.r.t. PMR write, and ensure they're visible
        // before potential out of band interrupt trigger because of PMR update.
        dmbishst();
        self.gicc_write_pmr(mask);
        dsbishst();

        old_mask
    }

    // This function updates single interrupt configuration to be level/edge
    // triggered
    pub fn interrupt_set_cfg(&self, id: u32, cfg: InterruptCfg) {
        self.gicd_set_icfgr(id, cfg);
    }

    // ------------------------------------------------------------------------
    // Helper functions

    /// Check GICv2
    fn check_v2(&self) {
        // Ensure that this is a GICv2 system
        let gic_version =
            (self.gicd_read_pidr2() >> crate::PIDR2_ARCH_REV_SHIFT) & crate::PIDR2_ARCH_REV_MASK;

        assert!(gic_version == crate::ARCH_REV_GICV2 || gic_version == crate::ARCH_REV_GICV1);
    }

    fn spis_configure_defaults(&self) {
        let num_ints = self.gicd_read_typer() & crate::TYPER_IT_LINES_NO_MASK;
        let num_ints = (num_ints + 1) << 5;

        // Treat all SPIs as G1NS by default. The number of interrupts is
        // calculated as 32 * (IT_LINES + 1). We do 32 at a time.
        for index in (crate::MIN_SPI_ID..num_ints).step_by(32) {
            self.gicd_write_igroupr(index, 0);
        }

        // Setup the default SPI priorities doing four at a time
        for index in (crate::MIN_SPI_ID..num_ints).step_by(4) {
            self.gicd_write_ipriorityr(index, crate::GICD_IPRIORITYR_DEF_VAL);
        }

        // Treat all SPIs as level triggered by default, 16 at a time
        for index in (crate::MIN_SPI_ID..num_ints).step_by(16) {
            self.gicd_write_icfgr(index, 0);
        }
    }

    /// Helper function to configure properties of secure G0 SPIs.
    fn secure_spis_configure_props(&self, props: &[crate::InterruptProp]) {
        for prop in props {
            if prop.inter_num < crate::MIN_SPI_ID as u16 {
                continue;
            }

            // Configure this interrupt as a secure interrupt
            assert!(prop.inter_grp == crate::InterruptGrp::Group0);
            let inter_num = prop.inter_num as u32;
            self.gicd_clr_igroupr(inter_num);

            // Set the priority of this interrupt
            self.gicd_set_ipriorityr(inter_num, prop.inter_pri);

            // Target the secure interrupts to primary CPU
            self.gicd_set_itargetsr(inter_num, Target::CPU0);

            // Set interrupt configuration
            self.gicd_set_icfgr(inter_num, prop.inter_cfg);

            // Enable this interrupt
            self.gicd_set_isenabler(inter_num);
        }
    }

    /// Helper function to configure properties of secure G0 SGIs and PPIs.
    fn secure_ppi_sgi_setup_props(&self, props: &[crate::InterruptProp]) {
        // Disable all SGIs (imp. def.)/PPIs before configuring them. This is a
        // more scalable approach as it avoids clearing the enable bits in the
        // GICD_CTLR.
        self.gicd_write_icenabler(0, 0);

        // Setup the default PPI/SGI priorities doing four at a time
        for i in 0..crate::MIN_SPI_ID {
            self.gicd_write_ipriorityr(i, crate::GICD_IPRIORITYR_DEF_VAL);
        }

        let mut sec_ppi_sgi_mask = 0;
        for prop in props {
            if prop.inter_num >= crate::MIN_SPI_ID as u16 {
                continue;
            }

            // Configure this interrupt as a secure interrupt
            assert!(prop.inter_grp == crate::InterruptGrp::Group0);

            // Set interrupt configuration for PPIs. Configuration for SGIs
            // are ignored.
            let inter_num = prop.inter_num as u32;
            if (crate::MIN_PPI_ID..crate::MIN_SPI_ID).contains(&inter_num) {
                self.gicd_set_icfgr(inter_num, prop.inter_cfg);
            }

            // We have an SGI or a PPI. They are Group0 at reset
            sec_ppi_sgi_mask |= 1 << prop.inter_num;

            // Set the priority of this interrupt
            self.gicd_set_ipriorityr(inter_num, prop.inter_pri);
        }

        // Invert the bitmask to create a mask for non-secure PPIs and SGIs.
        // Program the GICD_IGROUPR0 with this bit mask.
        self.gicd_write_igroupr(0, !sec_ppi_sgi_mask);

        // Enable the Group 0 SGIs and PPIs
        self.gicd_write_isenabler(0, sec_ppi_sgi_mask);
    }

    // ------------------------------------------------------------------------
    // GICD read

    /// GIC Distributor interface accessors for reading entire registers
    fn gicd_read_pidr2(&self) -> u32 {
        unsafe { read_volatile(to_ptr(self.gicd_base, GICD_PIDR2_GICV2)) }
    }

    /// Accessor to read the GIC Distributor IGROUPR corresponding to the interrupt
    /// `id`, 32 interrupt ids at a time.
    fn gicd_read_igroupr(&self, id: u32) -> u32 {
        let n = (id >> crate::IGROUPR_SHIFT) as usize;
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_IGROUPR + (n << 2))) }
    }

    /// Accessor to read the GIC Distributor ISACTIVER corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_read_isactiver(&self, id: u32) -> u32 {
        let n = (id >> crate::ISACTIVER_SHIFT) as usize;
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_ISACTIVER + (n << 2))) }
    }

    /// Accessor to read the GIC Distributor ICGFR corresponding to the
    /// interrupt `id`, 16 interrupt IDs at a time.
    fn gicd_read_icfgr(&self, id: u32) -> u32 {
        let n = (id >> crate::ICFGR_SHIFT) as usize;
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_ICFGR + (n << 2))) }
    }

    fn gicd_read_typer(&self) -> u32 {
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_TYPER)) }
    }

    fn gicd_read_ctlr(&self) -> GicdCtlrS {
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_CTLR)) }
    }

    // ------------------------------------------------------------------------
    // GICD write

    /// Accessor to write the GIC Distributor IGROUPR corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_write_igroupr(&self, id: u32, val: u32) {
        let n = (id >> crate::IGROUPR_SHIFT) as usize;
        unsafe { write_volatile(to_ptr(self.gicd_base, crate::GICD_IGROUPR + (n << 2)), val) };
    }

    /// Accessor to write the GIC Distributor ISENABLER corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_write_isenabler(&self, id: u32, val: u32) {
        let n = (id >> crate::ISENABLER_SHIFT) as usize;
        unsafe {
            write_volatile(
                to_ptr(self.gicd_base, crate::GICD_ISENABLER + (n << 2)),
                val,
            )
        }
    }

    /// Accessor to write the GIC Distributor ICENABLER corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_write_icenabler(&self, id: u32, val: u32) {
        let n = (id >> crate::ICENABLER_SHIFT) as usize;
        unsafe {
            write_volatile(
                to_ptr(self.gicd_base, crate::GICD_ICENABLER + (n << 2)),
                val,
            )
        };
    }

    /// Accessor to write the GIC Distributor ISPENDR corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_write_ispendr(&self, id: u32, val: u32) {
        let n = (id >> crate::ISPENDR_SHIFT) as usize;
        unsafe { write_volatile(to_ptr(self.gicd_base, crate::GICD_ISPENDR + (n << 2)), val) };
    }

    /// Accessor to write the GIC Distributor ICPENDR corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_write_icpendr(&self, id: u32, val: u32) {
        let n = (id >> crate::ICPENDR_SHIFT) as usize;
        unsafe { write_volatile(to_ptr(self.gicd_base, crate::GICD_ICPENDR + (n << 2)), val) };
    }

    /// Accessor to write the GIC Distributor IPRIORITYR corresponding to the
    /// interrupt `id`, 4 interrupt IDs at a time.
    fn gicd_write_ipriorityr(&self, id: u32, val: u32) {
        let n = (id >> crate::IPRIORITYR_SHIFT) as usize;
        unsafe {
            write_volatile(
                to_ptr(self.gicd_base, crate::GICD_IPRIORITYR + (n << 2)),
                val,
            )
        };
    }

    /// Accessor to write the GIC Distributor ICFGR corresponding to the
    /// interrupt `id`, 16 interrupt IDs at a time.
    fn gicd_write_icfgr(&self, id: u32, val: u32) {
        let n = (id >> crate::ICFGR_SHIFT) as usize;
        unsafe { write_volatile(to_ptr(self.gicd_base, crate::GICD_ICFGR + (n << 2)), val) };
    }

    fn gicd_write_ctlr(&self, val: GicdCtlrS) {
        unsafe { write_volatile(to_ptr(self.gicd_base, crate::GICD_CTLR), val) };
    }

    // ------------------------------------------------------------------------
    // GICD's API

    fn gicd_get_igroupr(&self, id: u32) -> crate::InterruptGrp {
        let bit_num = id & ((1 << crate::IGROUPR_SHIFT) - 1);
        let reg_val = self.gicd_read_igroupr(id);
        if (reg_val >> bit_num) & 1 == 0 {
            crate::InterruptGrp::Group0
        } else {
            crate::InterruptGrp::Group1
        }
    }

    fn gicd_set_igroupr(&self, id: u32) {
        let bit_num = id & ((1 << crate::IGROUPR_SHIFT) - 1);
        let reg_val = self.gicd_read_igroupr(id);
        self.gicd_write_igroupr(id, reg_val | (1 << bit_num));
    }

    fn gicd_clr_igroupr(&self, id: u32) {
        let bit_num = id & ((1 << crate::IGROUPR_SHIFT) - 1);
        let reg_val = self.gicd_read_igroupr(id);
        self.gicd_write_igroupr(id, reg_val & !(1 << bit_num));
    }

    fn gicd_set_itargetsr(&self, id: u32, target: Target) {
        unsafe { write_volatile(to_ptr(self.gicd_base, GICD_ITARGETSR + id as usize), target) };
    }

    fn gicd_set_isenabler(&self, id: u32) {
        let bit_num = id & ((1 << crate::ISENABLER_SHIFT) - 1);
        self.gicd_write_isenabler(id, 1 << bit_num);
    }

    fn gicd_set_icenabler(&self, id: u32) {
        let bit_num = id & ((1 << crate::ICENABLER_SHIFT) - 1);
        self.gicd_write_icenabler(id, 1 << bit_num);
    }

    fn gicd_set_ispendr(&self, id: u32) {
        let bit_num = id & ((1 << crate::ISPENDR_SHIFT) - 1);
        self.gicd_write_ispendr(id, 1 << bit_num);
    }

    fn gicd_set_icpendr(&self, id: u32) {
        let bit_num = id & ((1 << crate::ICPENDR_SHIFT) - 1);
        self.gicd_write_icpendr(id, 1 << bit_num);
    }

    fn gicd_get_isactiver(&self, id: u32) -> bool {
        let bit_num = id & ((1 << crate::ISACTIVER_SHIFT) - 1);
        let reg_val = self.gicd_read_isactiver(id);
        (reg_val >> bit_num) & 1 == 1
    }

    fn gicd_set_ipriorityr(&self, id: u32, pri: u8) {
        let val = pri;
        unsafe {
            write_volatile(
                to_ptr::<u8>(self.gicd_base, crate::GICD_IPRIORITYR + id as usize),
                val,
            )
        };
    }

    fn gicd_set_icfgr(&self, id: u32, cfg: InterruptCfg) {
        // Interrupt configuration is a 2-bit field
        let bit_num = id & ((1 << crate::ICFGR_SHIFT) - 1);
        let bit_shift = bit_num << 1;

        let mut reg_val = self.gicd_read_icfgr(id);

        // Clear the field, and insert required configuration
        reg_val &= !(crate::GIC_CFG_MASK << bit_shift);
        reg_val |= (cfg as u32) << bit_shift;

        self.gicd_write_icfgr(id, reg_val);
    }

    // ------------------------------------------------------------------------
    // GICC read

    fn gicc_read_ctlr(&self) -> GiccCtlrS {
        unsafe { read_volatile(to_ptr(self.gicc_base, GICC_CTLR)) }
    }

    fn gicc_read_pmr(&self) -> u32 {
        unsafe { read_volatile(to_ptr(self.gicc_base, GICC_PMR)) }
    }

    fn gicc_read_iar(&self) -> u32 {
        unsafe { read_volatile(to_ptr(self.gicc_base, GICC_IAR)) }
    }

    fn gicc_read_hppir(&self) -> u32 {
        unsafe { read_volatile(to_ptr(self.gicc_base, GICC_HPPIR)) }
    }

    fn gicc_read_ahppir(&self) -> u32 {
        unsafe { read_volatile(to_ptr(self.gicc_base, GICC_AHPPIR)) }
    }

    fn gicc_read_rpr(&self) -> u32 {
        unsafe { read_volatile(to_ptr(self.gicc_base, GICC_RPR)) }
    }

    // ------------------------------------------------------------------------
    // GICC write

    fn gicc_write_pmr(&self, val: u8) {
        unsafe { write_volatile(to_ptr(self.gicc_base, GICC_PMR), val as u32) };
    }

    fn gicc_write_ctlr(&self, val: GiccCtlrS) {
        unsafe { write_volatile(to_ptr(self.gicc_base, GICC_CTLR), val) };
    }

    fn gicc_write_eoir(&self, val: u32) {
        unsafe { write_volatile(to_ptr(self.gicc_base, GICC_EOIR), val) };
    }
}
