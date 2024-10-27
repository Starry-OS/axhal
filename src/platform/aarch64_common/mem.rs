use aarch64_cpu::{asm::barrier, registers::*};
use tock_registers::interfaces::{ReadWriteable, Writeable};

use crate::mem::virt_to_phys;
use crate::mem::{MemRegion, MemRegionFlags, PhysAddr};
use page_table_entry::aarch64::{MemAttr, A64PTE};
use page_table_entry::{GenericPTE, MappingFlags};

use either::{Either, Left, Right};

/// Returns platform-specific memory regions.
pub(crate) fn platform_regions(traverser:&crate::mem::MemTraverser) {
    // Feature, should registerd by user, should'n use hard coding
    if of::machin_name().is_some_and(|name| name.contains("raspi") || name.contains("phytiumpi")) {
            traverser.accept_one(&MemRegion {
                paddr: 0x0.into(),
                size: 0x1000,
                flags: MemRegionFlags::RESERVED | MemRegionFlags::READ | MemRegionFlags::WRITE,
                name: "spintable",
            });
    } 
    fdt_region().as_ref().inspect(|reg|{traverser.accept_one(reg);});

    traverser
    .than(&free_regions)
    .than(&crate::mem::default_mmio_regions);
}

fn split_region(region: MemRegion, region2: &MemRegion) -> impl Iterator<Item = MemRegion> {
    let start1 = region.paddr.as_usize();
    let end1 = region.paddr.as_usize() + region.size;

    let start2 = region2.paddr.as_usize();
    let end2 = region2.paddr.as_usize() + region2.size;

    // mem region include region2
    let iterator: Either<_, _> = if start1 <= start2 && end1 >= end2 {
        let head_region = MemRegion {
            paddr: region.paddr,
            size: start2 - start1,
            flags: MemRegionFlags::FREE | MemRegionFlags::READ | MemRegionFlags::WRITE,
            name: region.name,
        };
        let tail_region = MemRegion {
            paddr: PhysAddr::from(end2),
            size: end1 - end2,
            flags: MemRegionFlags::FREE | MemRegionFlags::READ | MemRegionFlags::WRITE,
            name: region.name,
        };
        let size_tup = (head_region.size, tail_region.size);
        // Top(down) left size < 4K, need drop
        match size_tup {
            (x, y) if x < 0x1000 && y < 0x1000 => panic!("no vailid left region"),
            (x, _) if x < 0x1000 => Right([tail_region]),
            (_, y) if y < 0x1000 => Right([head_region]),
            _ => Left([head_region, tail_region]),
        }
    } else {
        Right([region])
    };
    iterator.into_iter()
}

// Free mem regions equal memory minus kernel and fdt region
fn free_regions(traverser:&crate::mem::MemTraverser) {
    match of::memory_nodes(){
        Some(nodes) => {
            let all_mem = nodes.flat_map(|m| {
                m.regions().filter_map(|r| {
                    if r.size.unwrap() > 0 {
                        Some(MemRegion {
                            paddr: PhysAddr::from(r.starting_address as usize).align_up_4k(),
                            size: r.size.unwrap(),
                            flags: MemRegionFlags::FREE | MemRegionFlags::READ | MemRegionFlags::WRITE,
                            name: "free memory",
                        })
                    } else {
                        None
                    }
                })
            });
        
            let hack_k_region = MemRegion {
                paddr: virt_to_phys((_stext as usize).into()).align_up_4k(),
                size: _ekernel as usize - _stext as usize,
                flags: MemRegionFlags::FREE,
                name: "kernel memory",
            };
        
            let filter_kernel_mem = all_mem.flat_map(move |m| split_region(m, &hack_k_region));
            let _ = filter_kernel_mem.flat_map(move |m| split_region(m, fdt_region().as_ref().unwrap())).try_for_each(|reg|if traverser.accept_one(&reg){
                Ok(())
            }else{
             Err(())   
            });
        },
        None=>{
            traverser.than(&crate::mem::default_free_regions);
        }
    }
}

const FDT_FIX_SIZE: usize = 0x10_0000; //1M
fn fdt_region() -> Option<MemRegion> {
    if of::fdt_available(){
        let fdt_ptr = of::get_fdt_ptr();
        Some(MemRegion {
            paddr: virt_to_phys((fdt_ptr.unwrap() as usize).into()).align_up_4k(),
            size: FDT_FIX_SIZE,
            flags: MemRegionFlags::RESERVED | MemRegionFlags::READ,
            name: "fdt reserved",
        })
    }else{
        None
    }
}

#[link_section = ".data.boot_page_table"]
static mut BOOT_PT_L0: [A64PTE; 512] = [A64PTE::empty(); 512];

#[link_section = ".data.boot_page_table"]
static mut BOOT_PT_L1: [A64PTE; 512] = [A64PTE::empty(); 512];

/// Initialize the MMU
///
/// # Safety
///
/// This function is unsafe because it directly manipulates the CPU registers.
pub unsafe fn init_mmu() {
    MAIR_EL1.set(MemAttr::MAIR_VALUE);

    // Enable TTBR0 and TTBR1 walks, page size = 4K, vaddr size = 48 bits, paddr size = 40 bits.
    let tcr_flags0 = TCR_EL1::EPD0::EnableTTBR0Walks
        + TCR_EL1::TG0::KiB_4
        + TCR_EL1::SH0::Inner
        + TCR_EL1::ORGN0::WriteBack_ReadAlloc_WriteAlloc_Cacheable
        + TCR_EL1::IRGN0::WriteBack_ReadAlloc_WriteAlloc_Cacheable
        + TCR_EL1::T0SZ.val(16);
    let tcr_flags1 = TCR_EL1::EPD1::EnableTTBR1Walks
        + TCR_EL1::TG1::KiB_4
        + TCR_EL1::SH1::Inner
        + TCR_EL1::ORGN1::WriteBack_ReadAlloc_WriteAlloc_Cacheable
        + TCR_EL1::IRGN1::WriteBack_ReadAlloc_WriteAlloc_Cacheable
        + TCR_EL1::T1SZ.val(16);
    TCR_EL1.write(TCR_EL1::IPS::Bits_48 + tcr_flags0 + tcr_flags1);
    barrier::isb(barrier::SY);

    // Set both TTBR0 and TTBR1
    let root_paddr = PhysAddr::from(BOOT_PT_L0.as_ptr() as usize).as_usize() as _;
    TTBR0_EL1.set(root_paddr);
    TTBR1_EL1.set(root_paddr);

    // Flush the entire TLB
    crate::arch::flush_tlb(None);

    put_debug();

    // Enable the MMU and turn on I-cache and D-cache
    SCTLR_EL1.modify(SCTLR_EL1::M::Enable + SCTLR_EL1::C::Cacheable + SCTLR_EL1::I::Cacheable);
    barrier::isb(barrier::SY);


    put_debug_paged();
}


#[cfg(all(target_arch = "aarch64"))]
unsafe fn put_debug() {
    use core::ptr;

    #[cfg(platform_family = "aarch64-phytiumpi")]
    {
        let state = (0x2800D018 as usize) as *mut u8;
        let put = (0x2800D000 as usize) as *mut u8;
        while (ptr::read_volatile(state) & (0x20 as u8)) != 0 {}
        *put = b'a';
    }
}

#[cfg(all(target_arch = "aarch64"))]
unsafe  fn put_debug_paged() {
    use core::ptr;
    #[cfg(platform_family = "aarch64-phytiumpi")]
    {
        let state = (0xFFFF00002800D018 as usize) as *mut u8;
        let put = (0xFFFF00002800D000 as usize) as *mut u8;
        while (ptr::read_volatile(state) & (0x20 as u8)) != 0 {}
        *put = b'p';
    }
}

const BOOT_MAP_SHIFT: usize = 30; // 1GB
const BOOT_MAP_SIZE: usize = 1 << BOOT_MAP_SHIFT; // 1GB

/// Map the kernel image to the virtual address space.
///
/// # Safety
///
/// The function is unsafe because it directly modify the page table entries by dereferencing raw pointers.
pub unsafe fn idmap_kernel(kernel_phys_addr: usize) {
    let aligned_address = (kernel_phys_addr) & !(BOOT_MAP_SIZE - 1);
    let l1_index = kernel_phys_addr >> BOOT_MAP_SHIFT;

    // 0x0000_0000_0000 ~ 0x0080_0000_0000, table
    BOOT_PT_L0[0] = A64PTE::new_table(PhysAddr::from(BOOT_PT_L1.as_ptr() as usize));
    // 1G block, kernel img
    BOOT_PT_L1[l1_index] = A64PTE::new_page(
        PhysAddr::from(aligned_address),
        MappingFlags::READ | MappingFlags::WRITE | MappingFlags::EXECUTE,
        true,
    );

//==============================

    BOOT_PT_L1[0] = A64PTE::new_page(
        PhysAddr::from(0),
        MappingFlags::READ | MappingFlags::WRITE | MappingFlags::DEVICE,
        true,
    );
    // // 0x0000_4000_0000..0x0000_8000_0000, 1G block, normal memory
    // boot_pt_l1[1] = A64PTE::new_page(
    //     PhysAddr::from(0x4000_0000),
    //     MappingFlags::READ | MappingFlags::WRITE | MappingFlags::EXECUTE,
    //     true,
    // );
    // 0x0000_8000_0000..0x0000_C000_0000, 2G block, normal memory
    BOOT_PT_L1[2] = A64PTE::new_page(
        PhysAddr::from(0x8000_0000),
        MappingFlags::READ | MappingFlags::WRITE | MappingFlags::EXECUTE,
        true,
    );
    // 0x0000_C000_0000..0x0001_0000_0000, 1G block, DEVICE memory
    // boot_pt_l1[3] = A64PTE::new_page(
    //     PhysAddr::from(0xc000_0000),
    //     MappingFlags::READ | MappingFlags::WRITE | MappingFlags::DEVICE,
    //     true,
    // );
}

/// Map a device with the given physical address to the page table.
///
/// # Safety
///
/// The function is unsafe because it directly modify the page table entries by dereferencing raw pointers.
pub unsafe fn idmap_device(phys_addr: usize) {
    let aligned_address = (phys_addr) & !(BOOT_MAP_SIZE - 1);
    let l1_index = phys_addr >> BOOT_MAP_SHIFT;
    if BOOT_PT_L1[l1_index].is_unused() {
        BOOT_PT_L1[l1_index] = A64PTE::new_page(
            PhysAddr::from(aligned_address),
            MappingFlags::READ | MappingFlags::WRITE | MappingFlags::DEVICE,
            true,
        );
    }
}

extern "C" {
    fn _stext();
    fn _ekernel();
}
