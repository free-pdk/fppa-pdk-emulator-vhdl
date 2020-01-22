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
library work;
use work.pdkpkg.all;
use work.txt_util.all;

entity pdkpwmg is
  generic (
    NAME: string
  );
  port (
    sysclk_i  : in std_logic;
    ihrc_i    : in std_logic;
    rst_i     : in std_logic;

    dat_i     : in wordtype;
    mask_i    : in wordtype;
    c_wen_i   : in std_logic; -- Control register write
    s_wen_i   : in std_logic; -- Scalar register write
    cubh_wen_i: in std_logic; -- Counter upper bound high register
    cubl_wen_i: in std_logic; -- Counter upper bound low register
    dth_wen_i : in std_logic; -- PWM duty value high register
    dtl_wen_i : in std_logic; -- PWM duty value low register

    -- Register values
    c_o       : out wordtype;
    s_o       : out wordtype;
    -- CUB/DT are read only.

    intr_o    : out std_logic;
    -- Timer output. 
    timout_o  : out std_logic;
    active_o  : out std_logic
  );
end entity pdkpwmg;

architecture sim of pdkpwmg is

  signal cnt_r    : unsigned(10 downto 0);
  signal c_r      : wordtype;
  signal s_r      : wordtype;
  signal cubh_r   : wordtype;
  signal cubl_r   : wordtype;
  signal dth_r    : wordtype;
  signal dtl_r    : wordtype;

  signal pclk_s   : std_logic;
  signal reload_s : std_logic;

  -- Shadow registers.
  signal cub_r    : unsigned(10 downto 0);
  signal dt_r     : unsigned(10 downto 0);
  signal clr_s    : std_logic;
  signal isclrwr_s: std_logic;
  signal pwmcmp_s : std_logic;

begin

  pclk_s <= sysclk_i when c_r(0)='0' else ihrc_i;

  -- Generate clr_s signal. This comes from a different clock domain.

  isclrwr_s <= c_wen_i and mask_i(4);
  sync: entity work.pulsesync
    port map (
      wclk_i  => sysclk_i,
      rst_i   => rst_i,
      en_i    => isclrwr_s,
      d_i     => dat_i(4),
      rclk_i  => pclk_s,
      d_o     => clr_s
    );

  
  Cinst: entity work.pdkreg
    generic map ( NAME => NAME & "C" )
    port map ( clk_i => sysclk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => c_wen_i, dat_o => c_r );

  Sinst: entity work.pdkreg
    generic map ( NAME => NAME & "S" )
    port map ( clk_i => sysclk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => s_wen_i, dat_o => s_r );

  CUBHinst: entity work.pdkreg
    generic map ( NAME => NAME & "CUBH" )
    port map ( clk_i => sysclk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => cubh_wen_i, dat_o => cubh_r );

  CUBLinst: entity work.pdkreg
    generic map ( NAME => NAME & "CUBL" )
    port map ( clk_i => sysclk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => cubl_wen_i, dat_o => cubl_r );

  DTHinst: entity work.pdkreg
    generic map ( NAME => NAME & "DTH" )
    port map ( clk_i => sysclk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => dth_wen_i, dat_o => dth_r );

  DTLinst: entity work.pdkreg
    generic map ( NAME => NAME & "DTL" )
    port map ( clk_i => sysclk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => dtl_wen_i, dat_o => dtl_r );

  reload_s <= '1' when cnt_r>=cub_r or c_r(7)='0' else '0';

  process(pclk_s)
  begin
    if rst_i='1' or c_r(7)='0' then
      cnt_r <= (others => '0');
    elsif rising_edge(pclk_s) then
      if clr_s='1' then
        cnt_r <= (others => '0');
      else
        if reload_s='1' then
          cnt_r     <= (others => '0');
        else
          cnt_r <= cnt_r + 1;
        end if;
      end if;
    end if;
  end process;

  process(pclk_s)
  begin
    if rst_i='1' then
      dt_r  <= (others => '0');
    elsif rising_edge(pclk_s) then
      if reload_s='1' then
        dt_r(7 downto 0) <= dtl_r;
        dt_r(10 downto 8) <= dth_r(2 downto 0);
      end if;
    end if;
  end process;

  process(pclk_s)
  begin
    if rst_i='1' then
      cub_r  <= (others => '0');
    elsif rising_edge(pclk_s) then
      if reload_s='1' then
        cub_r(7 downto 0)  <= cubl_r;
        cub_r(10 downto 8) <= cubh_r(2 downto 0);
      end if;
    end if;
  end process;

  -- Comparator

  pwmcmp_s <= '1' when cnt_r <= dt_r else '0';


  c_o <= c_r;
  s_o <= s_r;
  active_o  <= c_r(7);
  timout_o <= pwmcmp_s;


end sim;
