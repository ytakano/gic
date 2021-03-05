# Library for Arm's GIC, UNDER CONSTRUCTION

This library provides APIs for Arm's GIC.

## Initialization

### Primary CPU from secure world

```rust
use gic::v2::GICv2;

let gicv2 = GICv2::new(GICD_BASE, GICC_BASE);
let props = [];

// Initialize by primary CPU
// Enable the secure GIC distributor interface
// Enable SPIs, PPIs & SGIs
gicv2.distif_init(&props);

// Initialization of per CPU
// Enable the Group 0 SGIs and PPIs
gicv2.pcpu_distif_init(&props);

// Enable secure interrupts and use FIQs
// Disable legacy bypass
gicv2.cpuif_enable();
```

### Per CPU from secure world

```rust
use gic::v2::GICv2;

let gicv2 = GICv2::new(GICD_BASE, GICC_BASE);
let props = [];

// Initialization of per CPU
// Enable the Group 0 SGIs and PPIs
gicv2.pcpu_distif_init(&props);
```

## Build

nightly and aarch64's target are required.

 $ cargo +nightly build --target=aarch64-unknown-none-softfloat

Before building, you have to install the target as follows.

 $ rustup target add --toolchain nightly aarch64-unknown-none-softfloat
