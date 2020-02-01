--
--  FPPA PDK14 Microcontroller simulation model - Port simulation
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

entity pdkport is
  GENERIC (
    NAME: string
  );
  port (
    clk_i     : in std_ulogic;
    rst_i     : in std_ulogic;

    dat_i     : in wordtype;
    mask_i    : in wordtype;

    ctrlwen_i : in std_logic;
    pullwen_i : in std_logic;
    datwen_i  : in std_logic;

    ctrl_o    : out wordtype;
    pull_o    : out wordtype;
    pdata_o   : out wordtype;
    i0_rise_o : out std_logic;
    i0_fall_o : out std_logic;


    pad_i     : in std_logic_vector(7 downto 0);
    pad_o     : out std_logic_vector(7 downto 0)
  );
end entity pdkport;

architecture sim of pdkport is

  signal C_s    : wordtype;
  signal PH_s   : wordtype;
  signal Dout_s : wordtype;
  signal Din_r  : std_logic_vector(7 downto 0);
  signal Dq1_r  : std_logic_vector(7 downto 0);
  signal p0_qr  : std_logic;

  --variable padv: std_logic;

begin

  -- Sync
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      Dq1_r <= to_stdlogicvector(to_bitvector(pad_i));
      Din_r <= Dq1_r;
      p0_qr <= Din_r(0);
    end if;
  end process;

  Cinst: entity work.pdkreg generic map ( NAME => NAME & "C" )
    port map ( clk_i => clk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => ctrlwen_i, dat_o => C_s );

  PHinst: entity work.pdkreg generic map ( NAME => NAME & "PH" )
    port map ( clk_i => clk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => pullwen_i, dat_o => PH_s );

  Dinst: entity work.pdkreg generic map ( NAME => NAME & "D" )
    port map ( clk_i => clk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => datwen_i, dat_o => Dout_s );


  l1: for i in 0 to 7 generate
    pad_o(i) <= Dout_s(i) when C_s(i)='1' else 'Z' when PH_s(i)='0' else 'H';
  end generate;

  i0_rise_o <= '1' when p0_qr='0' and Din_r(0)='1' else '0';
  i0_fall_o <= '1' when p0_qr='1' and Din_r(0)='0' else '0';

  ctrl_o  <= C_s;
  pull_o  <= PH_s;
  pdata_o <= unsigned(Din_r);

end sim;
