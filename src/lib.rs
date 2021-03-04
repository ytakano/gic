#![no_std]

pub mod v2;

#[macro_use]
extern crate bitflags;

use synctools::mcs::{MCSLock, MCSLockGuard};

/*******************************************************************************
 * GIC Distributor interface general definitions
 ******************************************************************************/
// Constants to categorise interrupts
pub(crate) const MIN_SGI_ID: usize = 0;
pub(crate) const MIN_SEC_SGI_ID: usize = 8;
pub(crate) const MIN_PPI_ID: usize = 16;
pub(crate) const MIN_SPI_ID: usize = 32;
pub(crate) const MAX_SPI_ID: usize = 1019;

/// Mask for the priority field common to all GIC interfaces
pub(crate) const GIC_PRI_MASK: u32 = 0xff;

/// Mask for the configuration field common to all GIC interfaces */
pub(crate) const GIC_CFG_MASK: u32 = 0x3;

// Highest possible interrupt priorities
pub(crate) const GIC_HIGHEST_SEC_PRIORITY: u32 = 0x00;
pub(crate) const GIC_HIGHEST_NS_PRIORITY: u32 = 0x80;

// Common GIC Distributor interface register offsets
pub(crate) const GICD_CTLR: usize = 0x0; // Distributor Control Register
pub(crate) const GICD_TYPER: usize = 0x4;
pub(crate) const GICD_IIDR: usize = 0x8;
pub(crate) const GICD_IGROUPR: usize = 0x80;
pub(crate) const GICD_ISENABLER: usize = 0x100;
pub(crate) const GICD_ICENABLER: usize = 0x180;
pub(crate) const GICD_ISPENDR: usize = 0x200;
pub(crate) const GICD_ICPENDR: usize = 0x280;
pub(crate) const GICD_ISACTIVER: usize = 0x300;
pub(crate) const GICD_ICACTIVER: usize = 0x380;
pub(crate) const GICD_IPRIORITYR: usize = 0x400;
pub(crate) const GICD_ICFGR: usize = 0xc00; // Interrupt Configuration Registers
pub(crate) const GICD_NSACR: usize = 0xe00;

pub(crate) const IGROUPR_SHIFT: usize = 5;
pub(crate) const ISENABLER_SHIFT: usize = 5;
pub(crate) const ICENABLER_SHIFT: usize = ISENABLER_SHIFT;
pub(crate) const ISPENDR_SHIFT: usize = 5;
pub(crate) const ICPENDR_SHIFT: usize = ISPENDR_SHIFT;
pub(crate) const ISACTIVER_SHIFT: usize = 5;
pub(crate) const ICACTIVER_SHIFT: usize = ISACTIVER_SHIFT;
pub(crate) const IPRIORITYR_SHIFT: usize = 2;
pub(crate) const ITARGETSR_SHIFT: usize = 2;
pub(crate) const ICFGR_SHIFT: usize = 4;
pub(crate) const NSACR_SHIFT: usize = 4;

// GICD_TYPER shifts and masks
pub(crate) const TYPER_IT_LINES_NO_SHIFT: u32 = 0;
pub(crate) const TYPER_IT_LINES_NO_MASK: u32 = 0x1f;

/// Value used to initialize Normal world interrupt priorities four at a time
pub(crate) const GICD_IPRIORITYR_DEF_VAL: u32 = GIC_HIGHEST_NS_PRIORITY
    | (GIC_HIGHEST_NS_PRIORITY << 8)
    | (GIC_HIGHEST_NS_PRIORITY << 16)
    | (GIC_HIGHEST_NS_PRIORITY << 24);

#[derive(PartialEq, Clone, Copy)]
pub enum InterruptGrp {
    Group0,
    Group1,
}

#[derive(Clone, Copy)]
pub enum InterruptCfg {
    LevelSensitive = 0,
    EdgeTrigger = 0b10,
}

#[derive(Clone, Copy)]
pub struct InterruptProp {
    inter_num: u16,
    inter_pri: u8,
    inter_grp: InterruptGrp,
    inter_cfg: InterruptCfg,
}

static mut LOCK: MCSLock<()> = MCSLock::new(());

pub(crate) fn lock() -> MCSLockGuard<'static, ()> {
    unsafe { LOCK.lock() }
}

pub(crate) fn to_ptr<T>(base: usize, offset: usize) -> *mut T {
    (base + offset) as *mut T
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
