--
--  FPPA PDK14 Microcontroller simulation model - Simple register model
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
--use IEEE.STD_LOGIC_signed.all;
use ieee.numeric_std.all;
library work;
use work.txt_util.all;
use work.pdkpkg.all;

entity pdkreg is
  generic(
    RESET : wordtype := (others => '0');
    NAME: string
  );
  port (
    clk_i   : in std_ulogic;
    rst_i   : in std_ulogic;
    dat_i   : in wordtype;
    mask_i  : in wordtype;
    wen_i   : in std_logic;
    dat_o   : out wordtype
  );
end entity pdkreg;

architecture sim of pdkreg is

  signal r: wordtype;

begin
  process( clk_i, rst_i )
  begin
    if rst_i='1' then
      r <= RESET;
    elsif rising_edge(clk_i) then
      if wen_i='1' then
        report "SFR " & NAME & " write, data " & hstr(std_logic_vector(dat_i)) & " mask " & hstr(std_logic_vector(mask_i));
        r <= (r and not mask_i) or (dat_i AND mask_i);
      end if;
    end if;
  end process;

  dat_o <= r;

end sim;
