--
--  FPPA PDK14 Microcontroller simulation model - PWMG model
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
use ieee.numeric_std.all;

entity pulsesync is
  port (
    wclk_i  : in std_logic;
    rst_i   : in std_logic; -- Async
    d_i     : in std_logic;
    en_i    : in std_logic;

    rclk_i  : in std_logic;
    d_o     : out std_logic
  );
end entity pulsesync;


architecture sim of pulsesync is

  signal rq1_r, wq1_r, wq2_r: std_logic;

begin

  process(wclk_i)
  begin
    if rst_i='1' then
      rq1_r <= '0';
    elsif rising_edge(wclk_i) then
      if en_i='1' then
        rq1_r <= rq1_r xor d_i;
      end if;
    end if;
  end process;

  process(rclk_i)
  begin
    if rst_i='1' then
      wq1_r <= '0';
      wq2_r <= '0';
    elsif rising_edge(rclk_i) then
      wq1_r <= rq1_r;
      wq2_r <= wq1_r;
    end if;
  end process;

  d_o <= wq1_r xor wq2_r;

end sim;
