------- VHDL FPPA PDK simulation model ------

This project aims to provide a fully functional, timing accurate VHDL model for
simulating PAKAUK FPPA microcontrollers.

It is still a work in progress.

Current cores implemented:

	PDK14

Current chips supported:

	PFS154

-------------------------------- Open questions ----------------------------------

The following questions are yet to be answered:

Q1: Is a "T" cycle a clock cycle, or do instructions take 2 clocks to execute ?
    The fact that CPU sysclk is at most IHRC/2 seems to point in this direction,
    however it can also be provided by ILRC and EOSC without a "divider". As of
    now this prevents implementation of the design (only simulation supported).
    Some suggest that there might be a PLL inside the chip.
    
Q2: What is the priority for alternate functions on pins ? For example, on PFS154
    PA3 can either be GPIO, TM2PWM or PG2PWM. It is known that GPIO has the 
    lowest priority, but if both TIM2 and PWMG2 are configured for PA3 output, 
    which one gets outputted ? This can be found out by testing a physical chip.
    
Q3: How are inputs synchronized ? For GPIO the design currently uses 2 FF's 
    clocked by sysclk, but actual implementation is unknown. The synchronization
    is mandatory to avoid metastability issues.
    
Q4: How exactly are registers synchronized between the diverse clock domains ?
    What is the delay between, let's say, setting PWM0GC.4 and actually resetting
    the PWM counter?
    
Q5: When switching clocks across the chip, how is the clock output handled to 
    avoid narrow pulses thay might occur ? As of now, a simple multiplexer is 
    used for simulation.

