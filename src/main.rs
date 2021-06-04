// This program has utilities to read/understand the Intel Power and Thermal Management.
// Information taken from:
//
// Intel® 64 and IA-32 Architectures
// Software Developer’s Manual
// Volume 3 (3A, 3B, 3C & 3D):
// System Programming Guide
// Chapter 14: Power and Thermal Management.
//
// You will need to load the msr and cpuid kernel modules:
// # modprobe cpuid
// # modprobe msr
//
// And execute this program as root.
#[cfg(target_arch = "x86_64")]
use core::arch::x86_64;
use nix::fcntl;
use nix::sys::stat;
use nix::sys::uio;
use std::convert::TryInto;
extern crate num_cpus;

mod msrs {
    // These MSR Values are only valid for my CPU. To check the values for your processor, check:
    //
    // Intel® 64 and IA-32 Architectures
    // Software Developer’s Manual
    // Volume 4:
    // Model-Specific Registers
    pub const IA32_MPERF: i64 = 0xE7; // Maximum Performance Frequency Clock Count (RW)
    pub const IA32_APERF: i64 = 0xE8; // Actual Performance Frequency Clock Count (RW)
    pub const IA32_PERF_STATUS: i64 = 0x198; // Current Performance Status (RO)
    pub const IA32_PERF_CTL: i64 = 0x199; // Performance Control MSR (R/W)
    pub const IA32_THERM_STATUS: i64 = 0x19C; // Thermal Status Information (RO)
    pub const IA32_MISC_ENABLE: i64 = 0x1A0; // Enable Misc. Processor Features (R/W)
    pub const IA32_ENERGY_PERF_BIAS: i64 = 0x1B0; // Performance Energy Bias Hint (R/W)
    pub const MSR_CORE_PERF_LIMIT_REASONS: i64 = 0x64F; // Indicator of Frequency Clipping in Processor Cores (RO)
    pub const IA32_PM_ENABLE: i64 = 0x770; // Enable/Disable HWP (R/W)
    pub const IA32_HWP_CAPABILITIES: i64 = 0x771; // HWP Performance Range Enumeration (RO)
    pub const IA32_HWP_INTERRUPT: i64 = 0x773; // Control HWP Native Interrupts (R/W)
    pub const IA32_HWP_REQUEST: i64 = 0x774; // Power Management Control Hints to a Logical Processor (R/W)
    pub const IA32_HWP_STATUS: i64 = 0x777; // Log bits indicating changes to Guaranteed & excursions to Minimum (R/W)

    #[derive(Debug)]
    pub struct MSRCorePerfLimitReasons {
        pub pochot_status: u8,
        pub thermal_status: u8,
        pub residency_state_regulatorion_status: u8,
        pub running_average_thermal_limit_status: u8,
        pub vr_therm_alert_status: u8,
        pub vr_thermal_design_current_status: u8,
        pub other_status: u8,
        pub pkg_pltfrm_level_power_limiting_pl1_status: u8,
        pub pkg_pltfrm_level_power_limiting_pl2_status: u8,
        pub max_turbo_limit_status: u8,
        pub turbo_transition_attenuation_status: u8,
        pub pochot_log: u8,
        pub thermal_log: u8,
        pub residency_state_regulatorion_log: u8,
        pub running_average_thermal_limit_log: u8,
        pub vr_therm_alert_log: u8,
        pub vr_thermal_design_current_log: u8,
        pub other_log: u8,
        pub pkg_pltfrm_level_power_limiting_pl1_log: u8,
        pub pkg_pltfrm_level_power_limiting_pl2_log: u8,
        pub max_turbo_limit_log: u8,
        pub turbo_transition_attenuation_log: u8,
    }

    pub fn build_msr_core_perf_limit_reasons(msr: u64) -> MSRCorePerfLimitReasons {
        MSRCorePerfLimitReasons {
            pochot_status: ((msr >> 0) & 1) as u8,
            thermal_status: ((msr >> 1) & 1) as u8,
            residency_state_regulatorion_status: ((msr >> 4) & 1) as u8,
            running_average_thermal_limit_status: ((msr >> 5) & 1) as u8,
            vr_therm_alert_status: ((msr >> 6) & 1) as u8,
            vr_thermal_design_current_status: ((msr >> 7) & 1) as u8,
            other_status: ((msr >> 8) & 1) as u8,
            pkg_pltfrm_level_power_limiting_pl1_status: ((msr >> 10) & 1) as u8,
            pkg_pltfrm_level_power_limiting_pl2_status: ((msr >> 11) & 1) as u8,
            max_turbo_limit_status: ((msr >> 12) & 1) as u8,
            turbo_transition_attenuation_status: ((msr >> 13) & 1) as u8,
            pochot_log: ((msr >> 16) & 1) as u8,
            thermal_log: ((msr >> 17) & 1) as u8,
            residency_state_regulatorion_log: ((msr >> 20) & 1) as u8,
            running_average_thermal_limit_log: ((msr >> 21) & 1) as u8,
            vr_therm_alert_log: ((msr >> 22) & 1) as u8,
            vr_thermal_design_current_log: ((msr >> 23) & 1) as u8,
            other_log: ((msr >> 24) & 1) as u8,
            pkg_pltfrm_level_power_limiting_pl1_log: ((msr >> 26) & 1) as u8,
            pkg_pltfrm_level_power_limiting_pl2_log: ((msr >> 27) & 1) as u8,
            max_turbo_limit_log: ((msr >> 28) & 1) as u8,
            turbo_transition_attenuation_log: ((msr >> 29) & 1) as u8,
        }
    }

    #[derive(Debug)]
    pub struct IA32ThermStatus {
        pub thermal_status: u8,
        pub thermal_status_log: u8,
        pub prochot_forcepr_event: u8,
        pub prochot_forcepr_log: u8,
        pub critical_temperature_status: u8,
        pub critical_temperature_status_log: u8,
        pub thermal_threshold_1_status: u8,
        pub thermal_threshold_1_log: u8,
        pub thermal_threshold_2_status: u8,
        pub thermal_threshold_2_log: u8,
        pub power_limitation_status: u8,
        pub power_limitation_log: u8,
        pub current_limit_status: u8,
        pub current_limit_log: u8,
        pub cross_domain_limit_status: u8,
        pub cross_domain_limit_log: u8,
        pub digital_readout: u8,
        pub resolution_in_degrees_celsius: u8,
        pub reading_valid: u8,
    }

    pub fn build_ia32_therm_status(msr: u64) -> IA32ThermStatus {
        IA32ThermStatus {
            thermal_status: ((msr >> 0) & 1) as u8,
            thermal_status_log: ((msr >> 1) & 1) as u8,
            prochot_forcepr_event: ((msr >> 2) & 1) as u8,
            prochot_forcepr_log: ((msr >> 3) & 1) as u8,
            critical_temperature_status: ((msr >> 4) & 1) as u8,
            critical_temperature_status_log: ((msr >> 5) & 1) as u8,
            thermal_threshold_1_status: ((msr >> 6) & 1) as u8,
            thermal_threshold_1_log: ((msr >> 7) & 1) as u8,
            thermal_threshold_2_status: ((msr >> 8) & 1) as u8,
            thermal_threshold_2_log: ((msr >> 9) & 1) as u8,
            power_limitation_status: ((msr >> 10) & 1) as u8,
            power_limitation_log: ((msr >> 11) & 1) as u8,
            current_limit_status: ((msr >> 12) & 1) as u8,
            current_limit_log: ((msr >> 13) & 1) as u8,
            cross_domain_limit_status: ((msr >> 14) & 1) as u8,
            cross_domain_limit_log: ((msr >> 15) & 1) as u8,
            digital_readout: ((msr >> 16) & 0x7F) as u8,
            resolution_in_degrees_celsius: ((msr >> 27) & 0xF) as u8,
            reading_valid: ((msr >> 31) & 1) as u8,
        }
    }

    #[derive(Debug)]
    pub struct IA32HWPInterrupt {
        pub en_guaranteed_performance_change: u8,
        pub en_excursion_minimum: u8,
    }
    pub fn build_ia32_hwp_interrupt(msr: u64) -> IA32HWPInterrupt {
        IA32HWPInterrupt {
            en_guaranteed_performance_change: (msr & 1) as u8,
            en_excursion_minimum: ((msr >> 1) & 1) as u8,
        }
    }

    #[derive(Debug)]
    pub struct IA32HWPStatus {
        pub guaranteed_performance_change: u8,
        pub excursion_to_minimum: u8,
        pub highest_change: u8,
        pub peci_override_entry: u8,
        pub peci_override_exit: u8,
    }

    pub fn build_ia32_hwp_status(msr: u64) -> IA32HWPStatus {
        IA32HWPStatus {
            guaranteed_performance_change: (msr & 1) as u8,
            excursion_to_minimum: ((msr >> 2) & 1) as u8,
            highest_change: ((msr >> 3) & 1) as u8,
            peci_override_entry: ((msr >> 4) & 1) as u8,
            peci_override_exit: ((msr >> 5) & 1) as u8,
        }
    }

    #[derive(Debug)]
    pub struct IA32HWPRequest {
        pub minimum_performance: u8,
        pub maximum_performance: u8,
        pub desired_performance: u8,
        pub energy_performance_preference: u8,
        pub activity_window: u16,
        pub package_control: u8,
    }
    pub fn build_ia32_hwp_request(msr: u64) -> IA32HWPRequest {
        IA32HWPRequest {
            minimum_performance: (msr & 0xff) as u8,
            maximum_performance: ((msr >> 8) & 0xff) as u8,
            desired_performance: ((msr >> 16) & 0xff) as u8,
            energy_performance_preference: ((msr >> 24) & 0xff) as u8,
            activity_window: ((msr >> 32) & 0x3ff) as u16,
            package_control: ((msr >> 42) & 1) as u8,
        }
    }
    impl IA32HWPRequest {
        pub fn to_msr(&self) -> u64 {
            self.minimum_performance as u64
                | ((self.maximum_performance as u64) << 8)
                | ((self.desired_performance as u64) << 16)
                | ((self.energy_performance_preference as u64) << 24)
                | (((self.activity_window as u64) & 0x3FF) << 32)
                | (((self.package_control as u64) & 1) << 42)
        }
    }

    #[derive(Debug)]
    pub struct IA32HWPCapabilities {
        pub highest_performance: u8,
        pub guaranteed_performance: u8,
        pub most_efficient_performance: u8,
        pub lowest_performance: u8,
    }

    pub fn build_ia32_hwp_capabilities(msr: u64) -> IA32HWPCapabilities {
        IA32HWPCapabilities {
            highest_performance: (msr & 0xff) as u8,
            guaranteed_performance: ((msr >> 8) & 0xff) as u8,
            most_efficient_performance: ((msr >> 16) & 0xff) as u8,
            lowest_performance: ((msr >> 24) & 0xff) as u8,
        }
    }

    #[derive(Debug)]
    pub struct IA32PMEnable {
        pub hwp_enable: u8,
    }
    pub fn build_ia32_pm_enable(msr: u64) -> IA32PMEnable {
        IA32PMEnable {
            hwp_enable: (msr & 0x1) as u8,
        }
    }
    #[derive(Debug)]
    pub struct IA32EnergyPerfBias {
        pub power_policy_preference: u8,
    }
    pub fn build_ia32_energy_perf_bias(msr: u64) -> IA32EnergyPerfBias {
        IA32EnergyPerfBias {
            power_policy_preference: (msr & 0xF) as u8,
        }
    }

    #[derive(Debug)]
    pub struct IA32MiscEnable {
        pub fast_strings_enable: bool,
        pub automated_thermal_control_circuit_enable: bool,
        pub performance_monitoring_available: bool,
        pub branch_storage_unavailable: bool,
        pub processor_event_based_sampling_unavailable: bool,
        pub enhanced_intel_speedstep_technology_enable: bool,
        pub enable_monitor_fsm: bool,
        pub limit_cpuid_maxval: bool,
        pub xtpr_message_disable: bool,
        pub xd_bit_disable: bool,
        pub dcu_prefetcher_disable: bool,
        pub ida_disable: bool,
        pub ip_prefetcher_disable: bool,
    }

    pub fn build_ia32_misc_enable(msr: u64) -> IA32MiscEnable {
        IA32MiscEnable {
            fast_strings_enable: (msr & (1 << 0)) != 0,
            automated_thermal_control_circuit_enable: (msr & (1 << 3)) != 0,
            performance_monitoring_available: (msr & (1 << 7)) != 0,
            branch_storage_unavailable: (msr & (1 << 11)) != 0,
            processor_event_based_sampling_unavailable: (msr & (1 << 12)) != 0,
            enhanced_intel_speedstep_technology_enable: (msr & (1 << 16)) != 0,
            enable_monitor_fsm: (msr & (1 << 18)) != 0,
            limit_cpuid_maxval: (msr & (1 << 22)) != 0,
            xtpr_message_disable: (msr & (1 << 23)) != 0,
            xd_bit_disable: (msr & (1 << 34)) != 0,
            dcu_prefetcher_disable: (msr & (1 << 37)) != 0,
            ida_disable: (msr & (1 << 38)) != 0,
            ip_prefetcher_disable: (msr & (1 << 39)) != 0,
        }
    }

    #[derive(Debug)]
    pub struct IA32PerfCtl {
        pub target_performance_state_value: u16,
        pub ida_engage: bool,
    }
    pub fn build_ia32_perf_ctl(msr: u64) -> IA32PerfCtl {
        IA32PerfCtl {
            target_performance_state_value: (msr & 0xFFFF) as u16,
            ida_engage: (msr & (1 << 32)) != 0,
        }
    }

    #[derive(Debug)]
    pub struct IA32PerfStatus {
        pub current_performance_state_value: u16,
    }
    pub fn build_ia32_perf_status(msr: u64) -> IA32PerfStatus {
        IA32PerfStatus {
            current_performance_state_value: (msr & 0xFFFF) as u16,
        }
    }

    #[derive(Debug)]
    pub struct IA32MPerf {
        pub c0_mcnt: u64,
    }
    pub fn build_ia32_mperf(msr: u64) -> IA32MPerf {
        IA32MPerf { c0_mcnt: msr }
    }

    #[derive(Debug)]
    pub struct IA32APerf {
        pub c0_acnt: u64,
    }
    pub fn build_ia32_aperf(msr: u64) -> IA32APerf {
        IA32APerf { c0_acnt: msr }
    }
}

#[allow(dead_code)]
mod thermal_and_power_management_eax {
    pub const DIGITAL_TEMPERATURE_SENSOR: u32 = 1 << 0;
    pub const INTEL_TURBO_BOOST_TECHNOLOGY_AVAILABLE: u32 = 1 << 1;
    pub const ARAT: u32 = 1 << 2; // APIC-Timer-always-running support
    pub const PLN: u32 = 1 << 4; // power limit notification support
    pub const ECMD: u32 = 1 << 5; // Clock Modulation duty cycle extension support
    pub const PTM: u32 = 1 << 6; // Package Thermal Management support
    pub const HWP: u32 = 1 << 7; // HWP base registers support
    pub const HWP_NOTIFICATION: u32 = 1 << 8; // IA32_HWP_INTERRUPT MSR support
    pub const HWP_ACTIVITY_WINDOW: u32 = 1 << 9; // IA32_HWP_REQUEST[41:32] MSR support
    pub const HWP_ENERGY_PERFORMANCE_PREFERENCE: u32 = 1 << 10; // IA32_HWP_REQUEST[31:24] MSR support
    pub const HWP_PACKAGE_LEVEL_REQUEST: u32 = 1 << 11; // IA32_HWP_REQUEST_PKG MSR support
    pub const HDC: u32 = 1 << 13; // HDC base registers support
    pub const INTEL_TURBO_BOOST_MAX_TECHNOLOGY_3_AVAILABLE: u32 = 1 << 14;
    pub const HWP_CAPABILITIES: u32 = 1 << 15; // Highest Performance change support
    pub const HWP_PECI_OVERRIDE: u32 = 1 << 16;
    pub const FLEXIBLE_HWP: u32 = 1 << 17;
    pub const FAST_ACCESS_MODE_IA32_HWP_REQUEST_MSR: u32 = 1 << 18;
    pub const HW_FEEDBACK: u32 = 1 << 19;
    pub const IGNORING_IDLE_LOGICAL_PROCESSOR_HWP_REQUEST: u32 = 1 << 20;
}

#[allow(dead_code)]
mod thermal_and_power_management_ecx {
    pub const HARDWARE_COORDINATION_FEEDBACK_CAPABILITY: u32 = 1 << 0;
    pub const PERFORMANCE_ENERGY_BIAS_PREFERENCE: u32 = 1 << 3;
}

#[allow(dead_code)]
mod feature_information_ecx {
    pub const SSE3: u32 = 1 << 0; // SSE3 Extensions
    pub const PCMULQDQ: u32 = 1 << 1; // Carryless Multiplication
    pub const DTEST64: u32 = 1 << 2; // 64-bit DS Area
    pub const MONITOR: u32 = 1 << 3; // MONITOR/MWAIT
    pub const DS_CPL: u32 = 1 << 4; // CPL Qualified Debug Store
    pub const VMX: u32 = 1 << 5; // Virtual Machine Extensions
    pub const SMX: u32 = 1 << 6; // Safer Mode Extensions
    pub const EIST: u32 = 1 << 7; // Enhanced Intel SpeedStep® Technology
    pub const TM2: u32 = 1 << 8; // Thermal Monitor 2
    pub const SSSE3: u32 = 1 << 9; // SSSE3 Extensions
    pub const CNTX_ID: u32 = 1 << 10; // L1 Context ID
    pub const SDBG: u32 = 1 << 11;
    pub const FMA: u32 = 1 << 12; // Fused Multiply Add
    pub const CMPXCHG16B: u32 = 1 << 13;
    pub const XTPR_UPDATE_CONTROL: u32 = 1 << 14;
    pub const PDCM: u32 = 1 << 15; // Perf/Debug Capability MSR
    pub const PCID: u32 = 1 << 17; // Process-Context Identifiers
    pub const DCA: u32 = 1 << 18; // Direct Cache Access
    pub const SSE4_1: u32 = 1 << 19; // SSE 4.1
    pub const SSE4_2: u32 = 1 << 20; // SSE 4.2
    pub const X2APIC: u32 = 1 << 21;
    pub const MOVBE: u32 = 1 << 22;
    pub const POPCNT: u32 = 1 << 23;
    pub const TSC_DEADLINE: u32 = 1 << 24;
    pub const AES: u32 = 1 << 25;
    pub const XSAVE: u32 = 1 << 26;
    pub const OSXSAVE: u32 = 1 << 27;
    pub const AVX: u32 = 1 << 28;
    pub const F16C: u32 = 1 << 29;
    pub const RDRAND: u32 = 1 << 30;
}

fn is_speedstep_supported(cpu: usize) -> Result<bool, nix::Error> {
    let cpuid = read_cpuid(cpu, 0x01, 0x00)?;
    Ok((cpuid.ecx & feature_information_ecx::EIST) != 0)
}

fn is_intel_dynamic_acceleration_disabled(cpu: usize) -> Result<bool, nix::Error> {
    let msr = read_msr(cpu, msrs::IA32_MISC_ENABLE)?;
    Ok(msrs::build_ia32_misc_enable(msr).ida_disable)
}

fn is_hardware_coordination_feedback_supported(cpu: usize) -> Result<bool, nix::Error> {
    let cpuid = read_cpuid(cpu, 0x06, 0x00)?;
    Ok(
        (cpuid.ecx & thermal_and_power_management_ecx::HARDWARE_COORDINATION_FEEDBACK_CAPABILITY)
            != 0,
    )
}

fn is_speedstep_enabled(cpu: usize) -> Result<bool, nix::Error> {
    let msr = read_msr(cpu, msrs::IA32_MISC_ENABLE)?;
    Ok(msrs::build_ia32_misc_enable(msr).enhanced_intel_speedstep_technology_enable)
}

fn write_msr(cpu: usize, msr: i64, val: u64) -> Result<(), nix::Error> {
    let msr_path = format!("/dev/cpu/{}/msr", cpu);
    let offset = msr;
    // let mut buf: [u8; 8] = [0; 8];
    let raw_fd = fcntl::open(msr_path.as_str(), fcntl::OFlag::O_RDWR, stat::Mode::empty())?;

    let writen = uio::pwrite(raw_fd, &mut u64::to_le_bytes(val)[..], offset)?;
    assert!(writen == 8);

    Ok(())
}

fn read_msr(cpu: usize, msr: i64) -> Result<u64, nix::Error> {
    let msr_path = format!("/dev/cpu/{}/msr", cpu);
    let offset = msr;
    let mut buf: [u8; 8] = [0; 8];
    let raw_fd = fcntl::open(
        msr_path.as_str(),
        fcntl::OFlag::O_RDONLY,
        stat::Mode::empty(),
    )?;
    let read = uio::pread(raw_fd, &mut buf[..], offset)?;
    assert!(read == 8);

    Ok(u64::from_le_bytes(buf))
}

fn as_array4(slice: &[u8]) -> &[u8; 4] {
    slice.try_into().expect("slice with incorrect length")
}

fn read_cpuid(cpu: usize, eax: u32, ecx: u32) -> Result<x86_64::CpuidResult, nix::Error> {
    let cpuid_path = format!("/dev/cpu/{}/cpuid", cpu);

    let offset: i64 = i64::from(eax) | (i64::from(ecx) << 32);
    let mut buf: [u8; 16] = [0; 16];

    let raw_fd = fcntl::open(
        cpuid_path.as_str(),
        fcntl::OFlag::O_RDONLY,
        stat::Mode::empty(),
    )?;
    let read = uio::pread(raw_fd, &mut buf[..], offset)?;
    assert!(read == 16);

    Ok(x86_64::CpuidResult {
        eax: u32::from_le_bytes(*as_array4(&buf[0..4])),
        ebx: u32::from_le_bytes(*as_array4(&buf[4..8])),
        ecx: u32::from_le_bytes(*as_array4(&buf[8..12])),
        edx: u32::from_le_bytes(*as_array4(&buf[12..16])),
    })
}

fn print_perf_ratio(cpu: usize) {
    let mperf = msrs::build_ia32_mperf(read_msr(cpu, msrs::IA32_MPERF).unwrap());
    let aperf = msrs::build_ia32_aperf(read_msr(cpu, msrs::IA32_APERF).unwrap());

    println!("\tIA32_MPERF: {:?}", mperf);
    println!("\tIA32_APERF: {:?}", aperf);
    println!(
        "\tRatio: {}",
        (aperf.c0_acnt as f64) / (mperf.c0_mcnt as f64)
    );
}

fn print_thermal_and_power_management_cpuid(cpu: usize) {
    let cpuid = read_cpuid(cpu, 0x06, 0x00).unwrap();
    println!(
        "{:<60} {}",
        "Digital temperature sensor support:",
        cpuid.eax & thermal_and_power_management_eax::DIGITAL_TEMPERATURE_SENSOR != 0
    );
    println!(
        "{:<60} {}",
        "Intel® Turbo Boost Technology available:",
        cpuid.eax & thermal_and_power_management_eax::INTEL_TURBO_BOOST_TECHNOLOGY_AVAILABLE != 0
    );
    println!(
        "{:<60} {}",
        "ARAT. APIC-Timer-always running feature support:",
        cpuid.eax & thermal_and_power_management_eax::ARAT != 0
    );
    println!(
        "{:<60} {}",
        "PLN. Power limit notification controls support:",
        cpuid.eax & thermal_and_power_management_eax::PLN != 0
    );
    println!(
        "{:<60} {}",
        "EMCD. Clock modulation duty cycle extension support:",
        cpuid.eax & thermal_and_power_management_eax::ECMD != 0
    );
    println!(
        "{:<60} {}",
        "PTM. Package thermal management support:",
        cpuid.eax & thermal_and_power_management_eax::PTM != 0
    );
    println!(
        "{:<60} {}",
        "HWP Base Registers support:",
        cpuid.eax & thermal_and_power_management_eax::HWP != 0
    );
    println!(
        "{:<60} {}",
        "IA32_HWP_INTERRUPT MSR support:",
        cpuid.eax & thermal_and_power_management_eax::HWP_NOTIFICATION != 0
    );
    println!(
        "{:<60} {}",
        "IA32_HWP_REQUEST[41:32] MSR support:",
        cpuid.eax & thermal_and_power_management_eax::HWP_ACTIVITY_WINDOW != 0
    );
    println!(
        "{:<60} {}",
        "IA32_HWP_REQUEST[31:24] MSR support:",
        cpuid.eax & thermal_and_power_management_eax::HWP_ENERGY_PERFORMANCE_PREFERENCE != 0
    );
    println!(
        "{:<60} {}",
        "IA32_HWP_REQUEST_PKG MSR support:",
        cpuid.eax & thermal_and_power_management_eax::HWP_PACKAGE_LEVEL_REQUEST != 0
    );
    println!(
        "{:<60} {}",
        "HDC Base Register support:",
        cpuid.eax & thermal_and_power_management_eax::HDC != 0
    );
    println!(
        "{:<60} {}",
        "Intel® Turbo Boost Max Technology 3.0 available:",
        cpuid.eax & thermal_and_power_management_eax::INTEL_TURBO_BOOST_MAX_TECHNOLOGY_3_AVAILABLE
            != 0
    );
    println!(
        "{:<60} {}",
        "HWP Capabilities. Highes Performance change support:",
        cpuid.eax & thermal_and_power_management_eax::HWP_CAPABILITIES != 0
    );
    println!(
        "{:<60} {}",
        "HWP PECI Override support:",
        cpuid.eax & thermal_and_power_management_eax::HWP_PECI_OVERRIDE != 0
    );
    println!(
        "{:<60} {}",
        "Flexible HWP support:",
        cpuid.eax & thermal_and_power_management_eax::FLEXIBLE_HWP != 0
    );
    println!(
        "{:<60} {}",
        "Fast access mode for the IA32_HWP_REQUEST MSR support:",
        cpuid.eax & thermal_and_power_management_eax::FAST_ACCESS_MODE_IA32_HWP_REQUEST_MSR != 0
    );
    println!(
        "{:<60} {}",
        "HW_FEEDBACK support:",
        cpuid.eax & thermal_and_power_management_eax::HW_FEEDBACK != 0
    );
    println!(
        "{:<60} {}",
        "Ignoring IDLE Logical Processor HWP request support:",
        cpuid.eax & thermal_and_power_management_eax::IGNORING_IDLE_LOGICAL_PROCESSOR_HWP_REQUEST
            != 0
    );

    println!(
        "{:<60} {}",
        "Hardware Coordination Feedback support:",
        cpuid.ecx & thermal_and_power_management_ecx::PERFORMANCE_ENERGY_BIAS_PREFERENCE != 0
    );
    println!(
        "{:<60} {}",
        "Performance-energy bias preference support:",
        cpuid.ecx & thermal_and_power_management_ecx::PERFORMANCE_ENERGY_BIAS_PREFERENCE != 0
    );
}

fn main() {
    println!("Hello, world!");
    println!(
        "{:<60} {}",
        "SpeedStep support:",
        is_speedstep_supported(0).unwrap()
    );
    println!(
        "{:<60} {}",
        "SpeedStep enabled:",
        is_speedstep_enabled(0).unwrap()
    );
    println!(
        "{:<60} {}",
        "Hardware Coordination Feedback support:",
        is_hardware_coordination_feedback_supported(0).unwrap()
    );
    println!(
        "{:<60} {}",
        "Intel Dynamic Acceleration Disabled:",
        is_intel_dynamic_acceleration_disabled(0).unwrap()
    );
    print_thermal_and_power_management_cpuid(0);

    for cpu in 0..num_cpus::get() {
        println!("CPU {}", cpu);
        println!(
            "\t{:?}",
            msrs::build_ia32_perf_ctl(read_msr(cpu, msrs::IA32_PERF_CTL).unwrap())
        );
        println!(
            "\t{:?}",
            msrs::build_ia32_perf_status(read_msr(cpu, msrs::IA32_PERF_STATUS).unwrap())
        );
        println!(
            "\t{:?}",
            msrs::build_ia32_energy_perf_bias(read_msr(cpu, msrs::IA32_ENERGY_PERF_BIAS).unwrap())
        );
        println!(
            "\t{:?}",
            msrs::build_ia32_pm_enable(read_msr(cpu, msrs::IA32_PM_ENABLE).unwrap())
        );

        println!(
            "\t{:?}",
            msrs::build_ia32_hwp_status(read_msr(cpu, msrs::IA32_HWP_STATUS).unwrap())
        );
        println!(
            "\t{:?}",
            msrs::build_ia32_hwp_interrupt(read_msr(cpu, msrs::IA32_HWP_INTERRUPT).unwrap())
        );
        println!(
            "\t{:?}",
            msrs::build_ia32_therm_status(read_msr(cpu, msrs::IA32_THERM_STATUS).unwrap())
        );

        print_perf_ratio(cpu);

        let hwp_caps =
            msrs::build_ia32_hwp_capabilities(read_msr(cpu, msrs::IA32_HWP_CAPABILITIES).unwrap());
        println!("Capabilities\t{:?}", hwp_caps);

        let hwp_req = msrs::IA32HWPRequest {
            minimum_performance: hwp_caps.most_efficient_performance,
            maximum_performance: hwp_caps.highest_performance,
            desired_performance: 0,
            energy_performance_preference: 0x0f0,
            activity_window: 0,

            package_control: 0,
        }
        .to_msr();

        // Uncomment to make the request and change the frecuency.
        // write_msr(cpu, msrs::IA32_HWP_REQUEST, hwp_req).unwrap();

        println!(
            "\t{:?}",
            msrs::build_ia32_hwp_request(read_msr(cpu, msrs::IA32_HWP_REQUEST).unwrap())
        );
    }

    println!(
        "\t{:?}",
        msrs::build_msr_core_perf_limit_reasons(
            read_msr(0, msrs::MSR_CORE_PERF_LIMIT_REASONS).unwrap()
        )
    );
}
