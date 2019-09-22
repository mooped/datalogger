/*
 * ESP32 datalogger ULP application
 *
 * Steve Barnett 2019
 *
 * Tracks reboot cycles of the main app and signals when CCS811 baseline
 * should be saved/restored for best results
 *
*/

/* C preprocessor constants */

// Constants below based on the main program running every 120 seconds

#define DEBUG_ULP 1

// Only restore the stored baseline to the CCS811 after ~45 mins
// Docs say the typical warmup is 20-30 mins, erring slightly on the side of caution
//#define WARMUP_PERIOD 3
#define WARMUP_PERIOD 23

// Initially save baseline every 720 boot cycles (24 hours)
//#define INITIAL_BASELINE_PERIOD 3
#define INITIAL_BASELINE_PERIOD 720

// Later save the baseline every 5040 boot cycles
//#define LONG_BASELINE_PERIOD 7
#define LONG_BASELINE_PERIOD 5040

// Longer baseline save period happens every 7 days (20 baseline saves)
//#define INITIAL_NUM_PERIODS 3
#define INITIAL_NUM_PERIODS 20

// Offset to avoid issues with how the ULP handles signed numbers
#define ZERO_BIAS 6000

/* Zero initialised values */
        .bss

        // Holds a magic number to let the main process know the ULP processor is running
        // 0x300d when running
        .global ulp_running
ulp_running:
        .long 0

        // Incremented by main process on boot
        .global boot_counter
boot_counter:
        .long 0

        // Filled with a magic number to signal to the main process that is should save a baseline
        // 0xd003 to store baseline
        .global store_baseline
store_baseline:
        .long 0

        // Filled with a magic number to signal to the main process that it should restore the saved baseline
        // 0xd003 to restore baseline
        .global restore_baseline
restore_baseline:
        .long 0

        // Mirror of restore_baseline, but never cleared to indicate whether readings should be considered good data
        .global warmup_complete
warmup_complete:
        .long 0

        // Main process writes non-zero to signal the ULP program to clear store_baseline and restore_baseline
        // Also clears stored_baseline
        .global stored_baseline
stored_baseline:
        .long 0

        // Number of baseline storge periods that have passed
        .global period_counter
period_counter:
        .long 0

#if DEBUG_ULP
        // Results of subtracting boot_counter from baseline_period - for debugging
        .global subbc
subbc:
        .long 0

        // Results of subtracting baseline_period from INITIAL_NUM_PERIODS - for debugging
        .global subpc
subpc:
        .long 0
#endif

/* Initialised data */
        .data

        // Period after which the baseline should be stored
        .global baseline_period
baseline_period:
        .long INITIAL_BASELINE_PERIOD

/* Code */
        .text

        .global entry
entry:
        // Put a special constant in ulp_running to signal to the main process that we are alive
        MOVE r0, 0x300d
        MOVE r1, ulp_running
        ST r0, r1, 0

entryloop:
        // Wait for WARMUP_PERIOD boot cycles before setting restore_baseline and entering the main loop
        MOVE r1, boot_counter
        LD r0, r1, 0
        JUMPR warmedup, WARMUP_PERIOD, GE

        // Go try again
        JUMP entryloop

warmedup:
        // Write magic number to restore_baseline and warmup_complete
        MOVE r0, 0xd003
        MOVE r1, restore_baseline
        MOVE r2, warmup_complete
        ST r0, r1, 0
        ST r0, r2, 0

        // Clear boot_counter
        MOVE r0, 0
        MOVE r1, boot_counter
        ST r0, r1, 0

mainloop:
        // Check if the main process has requested for store_baseline to be cleared
        MOVE r1, stored_baseline
        LD r0, r1, 0
        JUMPR clear, 1, GE
      
        // Subtract baseline_period from boot_counter + ZERO_BIAS
        // If greater than ZERO_BIAS trigger a baseline save
        // Hacky because the ULP support for signed integers seems weird!
        MOVE r1, baseline_period
        LD r0, r1, 0
        MOVE r2, boot_counter
        LD r3, r2, 0
        ADD r3, r3, ZERO_BIAS
        SUB r0, r3, r0
#if DEBUG_ULP
        MOVE r1, subbc
        ST r0, r1, 0
#endif
        JUMPR store, ZERO_BIAS, GE

        // Repeat
        JUMP mainloop

store:
        // Signal the main process to save the baseline register to Flash
        // Main process will load the value from Flash if it starts up and the ULP process is not running
        MOVE r1, store_baseline
        MOVE r0, 0xd003
        ST r0, r1, 0

        // Zero boot counter
        MOVE r1, boot_counter
        MOVE r0, 0x0000
        ST r0, r1, 0

        // Increment period counter
        MOVE r1, period_counter
        LD r0, r1, 0
        ADD r0, r0, 1
        ST r0, r1, 0

        // If this is the last cycle at the initial period, save the longer period

        // Subtract INITAL_NUM_PERIODS from period_counter + ZERO_BIAS
        // If greater than ZERO_BIAS move to longer baseline save periods
        // Hacky because the ULP support for signed integers seems weird!
        MOVE r1, INITIAL_NUM_PERIODS
        ADD r0, r0, ZERO_BIAS
        SUB r0, r0, r1
#if DEBUG_ULP
        MOVE r1, subpc
        ST r0, r1, 0
#endif
        JUMPR increase_period, ZERO_BIAS, GE

        // Restart the program
        JUMP mainloop

clear:
        // Clear the store_baseline, restore_baseline, and stored_baseline signals
        MOVE r1, store_baseline
        MOVE r2, stored_baseline
        MOVE r3, restore_baseline
        MOVE r0, 0
        ST r0, r1, 0
        ST r0, r2, 0
        ST r0, r3, 0

        // Restart the program
        JUMP mainloop

increase_period:
        // Increase the period after which baseline is stored

        // Load the extended baseline period into r0
        MOVE r0, LONG_BASELINE_PERIOD
        // Store the extended baseline period into baseline_period
        MOVE r1, baseline_period
        ST r0, r1, 0

        // Zero period counter
        MOVE r1, period_counter
        MOVE r0, 0
        ST r0, r1, 0

        // Restart the program
        JUMP mainloop

