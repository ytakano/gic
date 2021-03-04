use crate::{to_ptr, InterruptCfg};
use core::ptr::{read_volatile, write_volatile};

/*******************************************************************************
 * GICv2 specific Distributor interface register offsets and constants.
 ******************************************************************************/
const GICD_ITARGETSR: usize = 0x800;
const GICD_SGIR: usize = 0xF00;
const GICD_CPENDSGIR: usize = 0xF10;
const GICD_SPENDSGIR: usize = 0xF20;
const GICD_PIDR2_GICV2: usize = 0xFE8;

const ITARGETSR_SHIFT: u32 = 2;
const GIC_TARGET_CPU_MASK: u32 = 0xff;

/*******************************************************************************
 * GICv2 specific CPU interface register offsets and constants.
 ******************************************************************************/
// Physical CPU Interface registers
const GICC_CTLR: usize = 0x0; // CPU Interface Control Register
const GICC_PMR: usize = 0x4; // Interrupt Priority Mask Register
const GICC_BPR: usize = 0x8;
const GICC_IAR: usize = 0xC;
const GICC_EOIR: usize = 0x10;
const GICC_RPR: usize = 0x14;
const GICC_HPPIR: usize = 0x18;
const GICC_AHPPIR: usize = 0x28;
const GICC_IIDR: usize = 0xFC;
const GICC_DIR: usize = 0x1000;
const GICC_PRIODROP: usize = GICC_EOIR;

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

    /// Per cpu gic distributor setup which will be done by all cpus after a cold
    /// boot/hotplug. This marks out the secure SPIs and PPIs & enables them.
    pub fn pcpu_distif_init(&self, props: &[crate::InterruptProp]) {
        self.secure_ppi_sgi_setup_props(props);

        // Enable G0 interrupts if not already
        let ctlr = self.gicd_read_ctlr();
        if !ctlr.contains(GicdCtlrS::ENABLE_GRP0) {
            self.gicd_write_ctlr(ctlr | GicdCtlrS::ENABLE_GRP0);
        }
    }

    /// Global gic distributor init which will be done by the primary cpu after a
    /// cold boot. It marks out the secure SPIs, PPIs & SGIs and enables them. It
    /// then enables the secure GIC distributor interface.
    pub fn distif_init(&self, props: &[crate::InterruptProp]) {
        // Disable the distributor before going further
        let ctlr = self.gicd_read_ctlr();
        self.gicd_write_ctlr(ctlr & !(GicdCtlrS::ENABLE_GRP0 | GicdCtlrS::ENABLE_GRP1));

        // Set the default attribute of all SPIs
        self.spis_configure_defaults();

        self.secure_spis_configure_props(props);

        // Re-enable the secure SPIs now that they have been configured
        self.gicd_write_ctlr(ctlr | GicdCtlrS::ENABLE_GRP0);
    }

    fn spis_configure_defaults(&self) {
        let num_ints = self.gicd_read_typer() & crate::TYPER_IT_LINES_NO_MASK;
        let num_ints = ((num_ints + 1) << 5) as usize;

        // Treat all SPIs as G1NS by default. The number of interrupts is
        // calculated as 32 * (IT_LINES + 1). We do 32 at a time.
        for index in (crate::MIN_SPI_ID..num_ints).step_by(32) {
            self.gicd_write_igroupr(index, 0);
        }

        // Setup the default SPI priorities doing four at a time
        for index in (crate::MIN_SPI_ID..num_ints).step_by(4) {
            self.gicd_write_iproirityr(index, crate::GICD_IPRIORITYR_DEF_VAL);
        }

        // Treat all SPIs as level triggered by default, 16 at a time
        for index in (crate::MIN_SPI_ID..num_ints).step_by(16) {
            self.gicd_write_icfgr(index, 0);
        }
    }

    /// Helper function to configure properties of secure G0 SPIs.
    fn secure_spis_configure_props(&self, props: &[crate::InterruptProp]) {
        for prop in props {
            if prop.inter_num >= crate::MIN_SPI_ID as u16 {
                continue;
            }

            // Configure this interrupt as a secure interrupt
            assert!(prop.inter_grp == crate::InterruptGrp::Group0);
            self.gicd_clr_igroupr(prop.inter_num as usize);

            // Set the priority of this interrupt
            self.gicd_set_ipriorityr(prop.inter_num as usize, prop.inter_pri);

            // Target the secure interrupts to primary CPU
            self.gicd_set_itargetsr(prop.inter_num as usize, self.get_cpuif_id());

            // Set interrupt configuration
            self.gicd_set_icfgr(prop.inter_num as usize, prop.inter_cfg);

            // Enable this interrupt
            self.gicd_set_isenabler(prop.inter_num as usize);
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
            self.gicd_write_iproirityr(i, crate::GICD_IPRIORITYR_DEF_VAL);
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
            if prop.inter_num >= crate::MIN_PPI_ID as u16
                && prop.inter_num < crate::MIN_SPI_ID as u16
            {
                self.gicd_set_icfgr(prop.inter_num as usize, prop.inter_cfg.clone());
            }

            // We have an SGI or a PPI. They are Group0 at reset
            sec_ppi_sgi_mask |= 1 << prop.inter_num;

            // Set the priority of this interrupt
            self.gicd_set_ipriorityr(prop.inter_num as usize, prop.inter_pri);
        }

        // Invert the bitmask to create a mask for non-secure PPIs and SGIs.
        // Program the GICD_IGROUPR0 with this bit mask.
        self.gicd_write_igroupr(0, !sec_ppi_sgi_mask);

        // Enable the Group 0 SGIs and PPIs
        self.gicd_write_isenabler(0, sec_ppi_sgi_mask);
    }

    /// Accessor to read the GIC Distributor ITARGETSR corresponding to the
    /// interrupt `id`, 4 interrupt IDs at a time.
    fn gicd_read_itargetsr(&self, id: usize) -> u32 {
        let n = id >> crate::ITARGETSR_SHIFT;
        unsafe { read_volatile(to_ptr(self.gicd_base, GICD_ITARGETSR + (n << 2))) }
    }

    /// Accessor to read the GIC Distributor IGROUPR corresponding to the interrupt
    /// `id`, 32 interrupt ids at a time.
    fn gicd_read_igroupr(&self, id: usize) -> u32 {
        let n = id >> crate::IGROUPR_SHIFT;
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_IGROUPR + (n << 2))) }
    }

    /// Accessor to read the GIC Distributor ICGFR corresponding to the
    /// interrupt `id`, 16 interrupt IDs at a time.
    fn gicd_read_icfgr(&self, id: usize) -> u32 {
        let n = id >> crate::ICFGR_SHIFT;
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_ICFGR + (n << 2))) }
    }

    fn gicd_read_typer(&self) -> u32 {
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_TYPER)) }
    }

    fn gicd_read_ctlr(&self) -> GicdCtlrS {
        unsafe { read_volatile(to_ptr(self.gicd_base, crate::GICD_CTLR)) }
    }

    fn gicc_read_ctlr(&self) -> GiccCtlrS {
        unsafe { read_volatile(to_ptr(self.gicc_base, GICC_CTLR)) }
    }

    /// Accessor to write the GIC Distributor IGROUPR corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_write_igroupr(&self, id: usize, val: u32) {
        let n = id >> crate::IGROUPR_SHIFT;
        unsafe { write_volatile(to_ptr(self.gicd_base, crate::GICD_IGROUPR + (n << 2)), val) };
    }

    /// Accessor to write the GIC Distributor ISENABLER corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_write_isenabler(&self, id: usize, val: u32) {
        let n = id >> crate::ISENABLER_SHIFT;
        unsafe {
            write_volatile(
                to_ptr(self.gicd_base, crate::GICD_ISENABLER + (n << 2)),
                val,
            )
        }
    }

    /// Accessor to write the GIC Distributor ICENABLER corresponding to the
    /// interrupt `id`, 32 interrupt IDs at a time.
    fn gicd_write_icenabler(&self, id: usize, val: u32) {
        let n = id >> crate::ICENABLER_SHIFT;
        unsafe {
            write_volatile(
                to_ptr(self.gicd_base, crate::GICD_ICENABLER + (n << 2)),
                val,
            )
        };
    }

    /// Accessor to write the GIC Distributor IPRIORITYR corresponding to the
    /// interrupt `id`, 4 interrupt IDs at a time.
    fn gicd_write_iproirityr(&self, id: usize, val: u32) {
        let n = id >> crate::IPRIORITYR_SHIFT;
        unsafe {
            write_volatile(
                to_ptr(self.gicd_base, crate::GICD_IPRIORITYR + (n << 2)),
                val,
            )
        };
    }

    /// Accessor to write the GIC Distributor ICFGR corresponding to the
    /// interrupt `id`, 16 interrupt IDs at a time.
    fn gicd_write_icfgr(&self, id: usize, val: u32) {
        let n = id >> crate::ICFGR_SHIFT;
        unsafe { write_volatile(to_ptr(self.gicd_base, crate::GICD_ICFGR + (n << 2)), val) };
    }

    fn gicd_write_ctlr(&self, val: GicdCtlrS) {
        unsafe { write_volatile(to_ptr(self.gicd_base, crate::GICD_CTLR), val) };
    }

    fn gicc_write_pmr(&self, val: u8) {
        unsafe { write_volatile(to_ptr(self.gicc_base, GICC_PMR), val as u32) };
    }

    fn gicc_write_ctlr(&self, val: GiccCtlrS) {
        unsafe { write_volatile(to_ptr(self.gicc_base, GICC_CTLR), val) };
    }

    fn get_cpuif_id(&self) -> u32 {
        let val = self.gicd_read_itargetsr(0);
        val & GIC_TARGET_CPU_MASK
    }

    pub fn gicd_clr_igroupr(&self, id: usize) {
        let bit_num = id & ((1 << crate::IGROUPR_SHIFT) - 1);
        let reg_val = self.gicd_read_igroupr(id);
        self.gicd_write_igroupr(id, reg_val & !(1 << bit_num));
    }

    pub fn gicd_set_itargetsr(&self, id: usize, target: u32) {
        let val = target & GIC_TARGET_CPU_MASK;
        unsafe { write_volatile(to_ptr(self.gicd_base, GICD_ITARGETSR + id), val) };
    }

    pub fn gicd_set_isenabler(&self, id: usize) {
        let bit_num = id & ((1 << crate::ISENABLER_SHIFT) - 1);
        self.gicd_write_isenabler(id, 1 << bit_num);
    }

    pub fn gicd_set_ipriorityr(&self, id: usize, pri: u8) {
        let val = pri;
        unsafe { write_volatile(to_ptr(self.gicc_base, crate::GICD_IPRIORITYR + id), val) };
    }

    pub fn gicd_set_icfgr(&self, id: usize, cfg: InterruptCfg) {
        // Interrupt configuration is a 2-bit field
        let bit_num = id & ((1 << crate::ICFGR_SHIFT) - 1);
        let bit_shift = bit_num << 1;

        let mut reg_val = self.gicd_read_icfgr(id);

        // Clear the field, and insert required configuration
        reg_val &= !(crate::GIC_CFG_MASK << bit_shift);
        reg_val |= (cfg as u32) << bit_shift;

        self.gicd_write_icfgr(id, reg_val);
    }
}
