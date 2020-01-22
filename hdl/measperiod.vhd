--
--  FPPA PDK14 Microcontroller simulation model - Period measurement tool
--
--  Copyright 2020 Alvaro Lopes <alvieboy@alvie.com>
--
--  The FreeBSD license
--
--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions
--  are met:
--
--  1. Redistributions of source code must retain the above copyright
--     notice, this list of conditions and the following disclaimer.
--  2. Redistributions in binary form must reproduce the above
--     copyright notice, this list of conditions and the following
--     disclaimer in the documentation and/or other materials
--     provided with the distribution.
--
--  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY
--  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
--  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
--  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--  ZPU PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
--  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
--  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
--  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
--  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
--  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
--  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
--

library IEEE;
use IEEE.STD_LOGIC_1164.all;
library work;
use work.txt_util.all;

entity measperiod is
  generic (
    NAME: string
  );
  port (
    enabled_i : in boolean := true;
    sig_i     : in std_logic
  );
end entity measperiod;

architecture sim of measperiod is

begin

  process
    variable start  : time;
    variable delta  : time;
    variable fall   : time;
    variable ontime : time;
    variable deltaint, ontimeint: natural;
    variable percent: real;
    variable freqkhz: real;
  begin
    wait until rising_edge(sig_i);

    l1: loop
      start := now;
      wait until falling_edge(sig_i);
      fall := now;
      wait until rising_edge(sig_i);
      delta :=  (now - start);
      ontime := (fall-start);

      deltaint := delta / 1 ns;
      ontimeint := ontime / 1 ns;

      percent := real(ontimeint) * 100.0;
      percent := percent / real(deltaint);

      -- Convert delta to khz
      freqkhz := 1000000.0 / real(deltaint);
      report NAME & ": period " & str(deltaint) & " ns ("& str(freqkhz,2) &" kHz), on " & str(ontimeint) & " ns ("& str(percent,2)&"%)";


    end loop;
    wait;
  end process;

end sim;
