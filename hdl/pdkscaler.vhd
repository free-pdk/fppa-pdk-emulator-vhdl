--
--  FPPA PDK14 Microcontroller simulation model - Prescaler + scaler module
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
library work;
use work.pdkpkg.all;
use work.txt_util.all;

entity pdkscaler is
  port (
    clk_i   : in std_logic;
    rst_i   : in std_logic;
    pres_i  : in unsigned(1 downto 0);
    scal_i  : in unsigned(4 downto 0);

    clk_o   : out std_logic
  );
end pdkscaler;

architecture sim of pdkscaler is

  signal pres_r   : unsigned(5 downto 0);
  signal scal_r   : unsigned(4 downto 0);
  signal timpres_s: std_logic;
  signal clkgen_s : std_logic;

begin

  process(rst_i, clk_i)
  begin
    if rst_i='1' then
      pres_r <= (others =>'0');
    else
      if rising_edge(clk_i) then
        pres_r <= pres_r + 1;
      end if;
    end if;
  end process;

  with pres_i select timpres_s <=
    clk_i     when "00",
    pres_r(1) when "01",  -- /4
    pres_r(3) when "10",  -- /16
    pres_r(5) when "11",  -- /64
    '0' when others;

  -- Scalar...
  process(rst_i, timpres_s)
  begin
    if rst_i='1' then
      scal_r <= (others =>'0');
      clkgen_s <= '0';
    else
      if rising_edge(timpres_s) then
        if scal_r >= scal_i then
          scal_r <= (others => '0');
          clkgen_s <= not clkgen_s;
        else
          scal_r <= scal_r + 1;
        end if;
      end if;
    end if;
  end process;

  clk_o <= timpres_s when scal_i(4 downto 0)="00000" else clkgen_s;

end sim;
