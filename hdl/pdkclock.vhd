--
--  FPPA PDK14 Microcontroller simulation model - Clock system
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
use ieee.std_logic_misc.all;

library work;
use work.txt_util.all;
use work.pdkpkg.all;

entity pdkclock is
  generic (
    EOSC_CONNECTED: boolean := false
  );
  port (
    rst_i     : in std_logic;
    eosc_i    : in std_logic;
    clkmd_i   : in wordtype;
    sysclk_o  : out std_logic
  );
end entity pdkclock;

architecture sim of pdkclock is
  
  signal ihrc_s       : std_logic := '0';
  signal ilrc_s       : std_logic := '0';
  signal clksel_s     : unsigned(3 downto 0);
  signal eosc_s       : std_logic;

  signal ihrc_cnt_r   : unsigned(5 downto 0);
  signal ilrc_cnt_r   : unsigned(3 downto 0);
  signal eosc_cnt_r   : unsigned(2 downto 0);

  alias ihrc_div2_s   : std_logic is ihrc_cnt_r(0);
  alias ihrc_div4_s   : std_logic is ihrc_cnt_r(1);
  alias ihrc_div8_s   : std_logic is ihrc_cnt_r(2);
  alias ihrc_div16_s  : std_logic is ihrc_cnt_r(3);
  alias ihrc_div32_s  : std_logic is ihrc_cnt_r(4);
  alias ihrc_div64_s  : std_logic is ihrc_cnt_r(5);

  alias eosc_div2_s   : std_logic is eosc_cnt_r(0);
  alias eosc_div4_s   : std_logic is eosc_cnt_r(1);
  alias eosc_div8_s   : std_logic is eosc_cnt_r(2);

  alias ilrc_div2_s   : std_logic is ilrc_cnt_r(0);
  alias ilrc_div16_s  : std_logic is ilrc_cnt_r(3);

begin

  eosc_s <= eosc_i when EOSC_CONNECTED else '0';

  ilrc_s <= not ilrc_s AFTER ILRC_PERIOD / 2;
  ihrc_s <= not ihrc_s AFTER IHRC_PERIOD / 2;

  clksel_s <= clkmd_i(3) & clkmd_i(7 downto 5);

  with clksel_s select sysclk_o <=
    ihrc_div4_s   when "0000",
    ihrc_div2_s   when "0001",
    eosc_div4_s   when "0011",
    eosc_div2_s   when "0100",
    eosc_s        when "0101",
    ilrc_div2_s   when "0110",
    ilrc_s        when "0111",
    ihrc_div16_s  when "1000",
    ihrc_div8_s   when "1001",
    ilrc_div16_s  when "1010",
    ihrc_div32_s  when "1011",
    ihrc_div64_s  when "1100",
    eosc_div8_s   when "1101",
    '0' when others;


  -- Counters
  process (rst_i, ilrc_s)
  begin
    if rst_i='1' then
      ilrc_cnt_r <= (others => '0');
    elsif rising_edge(ilrc_s) then
      ilrc_cnt_r <= ilrc_cnt_r + 1;
    end if;
  end process;

  process (rst_i, ihrc_s)
  begin
    if rst_i='1' then
      ihrc_cnt_r <= (others => '0');
    elsif rising_edge(ihrc_s) then
      ihrc_cnt_r <= ihrc_cnt_r + 1;
    end if;
  end process;

  process (rst_i, eosc_s)
  begin
    if rst_i='1' then
      eosc_cnt_r <= (others => '0');
    elsif rising_edge(eosc_s) then
      eosc_cnt_r <= eosc_cnt_r + 1;
    end if;
  end process;



end sim;
