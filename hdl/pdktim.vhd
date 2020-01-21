--
--  FPPA PDK14 Microcontroller simulation model - Tim2/Tim3 model
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

entity pdktim is
  generic (
    NAME: string
  );
  port (
    clk_i     : in std_logic;
    rst_i     : in std_logic;

    ihrc_i    : in std_logic;
    eosc_i    : in std_logic;
    ilrc_i    : in std_logic;
    comp_i    : in std_logic;
    pa0_i     : in std_logic;
    pb0_i     : in std_logic;
    pa4_i     : in std_logic;

    dat_i     : in wordtype;
    mask_i    : in wordtype;
    c_wen_i   : in std_logic;
    s_wen_i   : in std_logic;
    ct_wen_i  : in std_logic;
    b_wen_i   : in std_logic;

    -- Register values
    c_o       : out wordtype;
    s_o       : out wordtype;
    ct_o      : out wordtype;
    b_o       : out wordtype;

    intr_o    : out std_logic;
    -- Timer output. NOTE: muxing into PB2/PA3/PA4 must be done externally
    timout_o  : out std_logic
  );
end entity pdktim;

architecture sim of pdktim is

  signal cnt_r    : wordtype;
  signal c_r      : wordtype;
  signal s_r      : wordtype;
  signal ct_r     : wordtype;
  signal b_r      : wordtype;
  signal timclk_s : std_logic;
  signal pa0n_s   : std_logic;
  signal pb0n_s   : std_logic;
  signal pa4n_s   : std_logic;
  signal pres_r   : unsigned(5 downto 0);
  signal scal_r   : unsigned(4 downto 0);
  signal timpres_s: std_logic; -- timer clock after prescaler
  signal tick_s   : std_logic;
  signal stick_s  : std_logic;
  signal event_s  : std_logic;
  signal toggle_r : std_logic;
  signal pwmcmp_s : std_logic;
begin

  Cinst: entity work.pdkreg
    generic map ( NAME => NAME & "C" )
    port map ( clk_i => clk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => c_wen_i, dat_o => c_r );

  Sinst: entity work.pdkreg
    generic map ( NAME => NAME & "S" )
    port map ( clk_i => clk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => s_wen_i, dat_o => s_r );

  CTinst: entity work.pdkreg
    generic map ( NAME => NAME & "CT" )
    port map ( clk_i => clk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => ct_wen_i, dat_o => ct_r );

  Binst: entity work.pdkreg
    generic map ( NAME => NAME & "B" )
    port map ( clk_i => clk_i, rst_i => rst_i, dat_i => dat_i, mask_i => mask_i, wen_i => b_wen_i, dat_o => b_r );


  pa0n_s  <= not pa0_i;
  pb0n_s  <= not pb0_i;
  pa4n_s  <= not pa4_i;

  -- Tim clock
  with c_r(7 downto 4) select timclk_s <=
    '0'     when "0000",
    clk_i   when "0001",
    ihrc_i  when "0010",
    eosc_i  when "0011",
    ilrc_i  when "0100",
    comp_i  when "0101",
    pa0_i   when "1000",
    pa0n_s  when "1001",
    pb0_i   when "1010",
    pb0n_s  when "1011",
    pa4_i   when "1100",
    pa4n_s  when "1101",
    '0'     when others;

  -- Prescaler

  process(rst_i, timclk_s)
  begin
    if rst_i='1' then
      pres_r <= (others =>'0');
    else
      if rising_edge(timclk_s) then
        pres_r <= pres_r + 1;
      end if;
    end if;
  end process;

  with s_r(6 downto 5) select timpres_s <=
    timclk_s  when "00",
    pres_r(1) when "01",  -- /4
    pres_r(3) when "10",  -- /16
    pres_r(5) when "11",  -- /64
    '0' when others;

  -- Scalar...
  process(rst_i, timpres_s)
  begin
    if rst_i='1' then
      scal_r <= (others =>'0');
      stick_s <= '0';
    else
      if rising_edge(timpres_s) then
        if scal_r >= s_r(4 downto 0) then
          scal_r <= (others => '0');
          stick_s <= not stick_s;
        else
          scal_r <= scal_r + 1;
        end if;
      end if;
    end if;
  end process;

  tick_s <= timpres_s when s_r(4 downto 0)="00000" else stick_s;


  process(rst_i, tick_s)
    variable resetcnt_v: boolean;
    variable pwmcomp_v: unsigned(7 downto 0);
  begin
    if rst_i='1' then
      cnt_r <= (others => '0');
      event_s<='0';
      toggle_r <= '0';
    elsif rising_edge(tick_s) then
      resetcnt_v := false;
      if c_r(1)='0' then
        -- Period mode
        if cnt_r >= b_r then
          resetcnt_v := true;
        end if;
      else
        -- PWM mode
        if s_r(7)='0' then
          -- 8-bit mode
          pwmcomp_v := x"FF";
        else
          -- 6-bit mode
          pwmcomp_v := x"3F";
        end if;
        if cnt_r >= pwmcomp_v then
          resetcnt_v := true;
        end if;
      end if;
      if resetcnt_v then
        cnt_r<=(others =>'0');
        event_s<='1';
        toggle_r <= not toggle_r;
      else
        cnt_r<=cnt_r + 1;
        event_s<='0';
        end if;
    end if;
  end process;

  b_o   <= b_r;
  c_o   <= c_r;
  ct_o  <= ct_r;
  s_o   <= s_r;

  pwmcmp_s <= '1' when cnt_r <= b_r else '0';

  timout_o <= (toggle_r xor c_r(0)) when c_r(1)='0' else (pwmcmp_s xor c_r(0));
  intr_o <= '1' when cnt_r = b_r else '0';

end sim;


