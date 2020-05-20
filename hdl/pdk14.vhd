--
--  FPPA PDK14 Microcontroller simulation model
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
use work.pdk14pkg.all;

entity pdk14 is
  generic (
    -- Setting this to true will enable EOSC input and disable PA6/PA7
    EOSC_CONNECTED: boolean := false;
    DEBUG_ENABLED: boolean := false
  );
  port (
    PA_io   : inout std_logic_vector(7 downto 0);
    PB_io   : inout std_logic_vector(7 downto 0);
    -- Special pins with analog behaviour.
    eosc_i  : in std_logic := 'X' -- External oscillator (not crystal).
  );
end entity pdk14;

architecture sim of pdk14 is

  --
  -- Local types
  --

  type mem_type is array(0 to 128) of wordtype;

  --
  -- Constants
  --

  constant SFR_INDEX_SP   : natural := 2;

  --
  -- Shared variables
  --

  shared variable mem     : mem_type;
  shared variable tim16set: std_logic := '0';

  --
  -- Signals and registers
  --

  signal opcode_s         : opcodetype;
  signal flags            : flags_type;
  signal A                :  wordtype;
  signal ginten           : std_logic;
  signal jmp              : std_logic;
  signal dbg_mem          : mem_type;
  signal opcode_valid     : std_logic := '0';
  signal rst_s            : std_ulogic := '0';
  signal sysclk_s         : std_ulogic := '0';
  signal ihrc_s           : std_logic;
  signal ilrc_s           : std_logic;
  signal eosc_s           : std_logic;
  signal PC               : pctype;
  signal IPC              : pctype;
  signal npc              : pctype;
  signal cycle            : unsigned(31 downto 0):=(others => '0');
  signal dec              : opdec_type;
  signal opcode_full_s    : std_logic_vector(15 downto 0);
  signal int_s            : std_logic := '0';
  signal tim16setvalue    : unsigned(15 downto 0);
  signal timer16clk       : std_ulogic;
  signal t16cnt           : unsigned(15 downto 0);
  signal pt16cnt          : unsigned(15 downto 0);
  signal intsrc_s         : std_logic_vector(7 downto 0);
  signal sfr_read         : wordtype;
  signal sfr_wdata        : wordtype;
  signal sfr_wmask        : wordtype;
  signal sfr_wen          : std_logic_vector(127 downto 0);

  signal SP_s             : wordtype;
  signal CLKMD_s          : wordtype;
  signal INTEN_s          : wordtype;
  signal INTRQ            : wordtype;
  signal T16M_s           : wordtype;
  signal INTEGS_s         : wordtype;
  signal PADIER_s         : wordtype;
  signal PBDIER_S         : wordtype;
  signal PA_s             : wordtype;
  signal PAC_s            : wordtype;
  signal PAPH_s           : wordtype;
  signal PB_s             : wordtype;
  signal PBC_s            : wordtype;
  signal PBPH_s           : wordtype;
  signal MISC_s           : wordtype;
  signal TM2B_s           : wordtype;
  signal TM2C_s           : wordtype;
  signal TM2CT_s          : wordtype;
  signal TM2S_s           : wordtype;
  signal TM3B_s           : wordtype;
  signal TM3C_s           : wordtype;
  signal TM3CT_s          : wordtype;
  signal TM3S_s           : wordtype;
  signal GPCC             : wordtype;
  signal GPCS             : wordtype;
  signal PWMG0C_s         : wordtype;
  signal PWMG0S_s         : wordtype;
  --signal PWMG0DTH         : wordtype;
  --signal PWMG0DTL         : wordtype;
  --signal PWMG0CUBH        : wordtype;
  --signal PWMG0CUBL        : wordtype;
  signal PWMG1C           : wordtype;
  signal PWMG1S           : wordtype;
  signal PWMG1DTH         : wordtype;
  signal PWMG1DTL         : wordtype;
  signal PWMG1CUBH        : wordtype;
  signal PWMG1CUBL        : wordtype;
  signal PWMG2C           : wordtype;
  signal PWMG2S           : wordtype;
  signal PWMG2DTH         : wordtype;
  signal PWMG2DTL         : wordtype;
  signal PWMG2CUBH        : wordtype;
  signal PWMG2CUBL        : wordtype;

  signal t16intr          : std_logic;
  signal intreq_s         : std_logic_vector(7 downto 0) := (others => '0'); -- Interrupt request
  signal pa0_fall_s       : std_logic;
  signal pa0_rise_s       : std_logic;
  signal pb0_fall_s       : std_logic;
  signal pb0_rise_s       : std_logic;
  signal stall_s          : std_logic;
  signal tim2out_s        : std_logic;
  signal tim3out_s        : std_logic;
  signal tim2int_s        : std_logic;
  signal tim3int_s        : std_logic;
  signal PA_i_s           : std_logic_vector(7 downto 0);
  signal PB_i_s           : std_logic_vector(7 downto 0);
  signal PA_o_s           : std_logic_vector(7 downto 0);
  signal PB_o_s           : std_logic_vector(7 downto 0);
  signal pwmg0int_s       : std_logic;
  signal pwmg0out_s       : std_logic;
  signal pwmg0active_s    : std_logic;

  --
  -- Function and procedure declarations
  --

  function signextend11(a: in wordtype) return unsigned is
    variable r: unsigned(10 downto 0);
  begin
    r(7 downto 0) := a;
    r(10 downto 8) := (others => a(7));
    return r;
  end signextend11;
  
  function neg(a: in wordtype) return unsigned is
    variable r: wordtype;
  begin
    r := not a;
    r := r + 1;
     return r;
  end function;

  procedure writeMem(address: in natural; data: in wordtype) is
  begin
    if DEBUG_ENABLED then
      report " WRITE address=" & str(address) & " value 0x" & hstr(std_logic_vector(data));
    end if;
    mem(address) := data;
  end procedure;

  impure function readMem(address: in natural) return wordtype is
    variable d: wordtype;
  begin
    if (address>mem'HIGH) then
      report "Out of bounds mem access " &str(address) & " pc 0x" & hstr(std_logic_vector(ipc));
    end if;
    d:=mem(address);
    if DEBUG_ENABLED then
      report " READ address=" & str(address) & " value 0x" & hstr(std_logic_vector(d));
    end if;
    return d;
  end function;


begin

  rst_s <= '0', '1' after 10 ps, '0' after 200 ns;

  clock_inst: entity work.pdkclock
  generic map (
    EOSC_CONNECTED => EOSC_CONNECTED
  )
  port map (
    rst_i     => rst_s,
    eosc_i    => eosc_i,
    clkmd_i   => CLKMD_s,
    sysclk_o  => sysclk_s,
    ihrc_o    => ihrc_s,
    ilrc_o    => ilrc_s,
    eosc_o    => eosc_s
  );

  -- Debugging purposes only, to be visible on waveform
  process(sysclk_s)
  begin
    dbg_mem <= mem;
  end process;

  SPinst: entity work.pdkreg generic map ( NAME => "SP" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(2), dat_o => SP_s );

  INTENinst: entity work.pdkreg generic map ( NAME => "INTEN" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(4), dat_o => INTEN_s );

  INTEGSinst: entity work.pdkreg generic map ( NAME => "INTEGS" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(12), dat_o => INTEGS_s );

  PADIERinst: entity work.pdkreg generic map ( NAME => "PADIER" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(13), dat_o => PADIER_s );

  PBDIERinst: entity work.pdkreg generic map ( NAME => "PBDIER" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(14), dat_o => PBDIER_s );

  T16Minst: entity work.pdkreg generic map ( NAME => "T16M" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(6), dat_o => T16M_s );

  CLKMDinst: entity work.pdkreg generic map ( NAME => "CLKMD", RESET => x"E0" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(3), dat_o => CLKMD_s );

  MISCinst: entity work.pdkreg generic map ( NAME => "MISC" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(8), dat_o => MISC_s );


  -- Ports
  PAinst: entity work.pdkport
    generic map (
      NAME => "PA"
    )
    port map (
    clk_i     => sysclk_s,
    rst_i     => rst_s,
    dat_i     => sfr_wdata,
    mask_i    => sfr_wmask,
    datwen_i  => sfr_wen(16),
    ctrlwen_i => sfr_wen(17),
    pullwen_i => sfr_wen(18),
    i0_rise_o => pa0_rise_s,
    i0_fall_o => pa0_fall_s,
    ctrl_o    => PAC_s,
    pull_o    => PAPH_s,
    pdata_o   => PA_s,
    pad_i     => PA_i_s,
    pad_o     => PA_o_s
  );

  PBinst: entity work.pdkport
    generic map (
      NAME => "PB"
    )
    port map (
    clk_i     => sysclk_s,
    rst_i     => rst_s,
    dat_i     => sfr_wdata,
    mask_i    => sfr_wmask,
    datwen_i  => sfr_wen(20),
    ctrlwen_i => sfr_wen(21),
    pullwen_i => sfr_wen(22),
    i0_rise_o => pb0_rise_s,
    i0_fall_o => pb0_fall_s,
    ctrl_o    => PBC_s,
    pull_o    => PBPH_s,
    pdata_o   => PB_s,
    pad_i     => PB_i_s,
    pad_o     => PB_o_s
  );

  -- Tim2
  tim2inst: entity work.pdktim
    generic map ( NAME => "TIM2" )
    port map (
      clk_i     => sysclk_s,
      rst_i     => rst_s,
      ihrc_i    => ihrc_s,
      eosc_i    => eosc_s,
      ilrc_i    => ilrc_s,
      comp_i    => '0',
      pa0_i     => PA_i_s(0),
      pb0_i     => PB_i_s(0),
      pa4_i     => PA_i_s(4),
  
      dat_i     => sfr_wdata,
      mask_i    => sfr_wmask,
      c_wen_i   => sfr_wen(28),
      s_wen_i   => sfr_wen(23),
      ct_wen_i  => sfr_wen(29), -- TBD.
      b_wen_i   => sfr_wen(9),
  
      c_o       => TM2C_s,
      s_o       => TM2S_s,
      ct_o      => TM2CT_s,
      b_o       => TM2B_s,
      timout_o  => tim2out_s,
      intr_o    => tim2int_s
  );
  intreq_s(6) <= tim2int_s;

  -- Tim3
  tim3inst: entity work.pdktim
    generic map ( NAME => "TIM3" )
    port map (
      clk_i     => sysclk_s,
      rst_i     => rst_s,
      ihrc_i    => ihrc_s,
      eosc_i    => eosc_s,
      ilrc_i    => ilrc_s,
      comp_i    => '0',
      pa0_i     => PA_i_s(0),
      pb0_i     => PB_i_s(0),
      pa4_i     => PA_i_s(4),
  
      dat_i     => sfr_wdata,
      mask_i    => sfr_wmask,
      c_wen_i   => sfr_wen(50),
      s_wen_i   => sfr_wen(52),
      ct_wen_i  => sfr_wen(51), -- TBD.
      b_wen_i   => sfr_wen(53),
  
      c_o       => TM3C_s,
      s_o       => TM3S_s,
      ct_o      => TM3CT_s,
      b_o       => TM3B_s,
      timout_o  => tim3out_s,
      intr_o    => tim3int_s
  );
  intreq_s(7) <= tim3int_s;

  -- PWMG0

  pwmg0_inst: entity work.pdkpwmg
    generic map (
      NAME  => "PWMG0"
    )
    port map (
      sysclk_i  => sysclk_s,
      ihrc_i    => ihrc_s,
      rst_i     => rst_s,
  
      dat_i     => sfr_wdata,
      mask_i    => sfr_wmask,
      c_wen_i   => sfr_wen(32),
      s_wen_i   => sfr_wen(33),
      cubh_wen_i=> sfr_wen(36),
      cubl_wen_i=> sfr_wen(37),
      dth_wen_i => sfr_wen(34),
      dtl_wen_i => sfr_wen(35),
  
      -- Register values
      c_o       => PWMG0C_s,
      s_o       => PWMG0S_s,
      -- CUB/DT are read only.
  
      intr_o    => pwmg0int_s,
      -- Timer output. 
      timout_o  => pwmg0out_s,
      active_o  => pwmg0active_s
    );



  -- PA0 interrupt
  process(padier_s(0), integs_s(1 downto 0), pa0_rise_s, pa0_fall_s)
  begin
    if padier_s(0)='1' then
      case integs_s(1 downto 0) is
        when "00" => intreq_s(0) <= pa0_rise_s OR pa0_fall_s;
        when "01" => intreq_s(0) <= pa0_rise_s;
        when "10" => intreq_s(0) <= pa0_fall_s;
        when "11" => intreq_s(0) <= '0';
        when others => intreq_s(0) <= '0';
      end case;
    else
      -- Disabled interrupt
      intreq_s(0) <= '0';
    end if;
  end process;

  -- PB0 interrupt
  process(pbdier_s(0), integs_s(3 downto 2), pb0_rise_s, pb0_fall_s)
  begin
    if pbdier_s(0)='1' then
      case integs_s(3 downto 2) is
        when "00" => intreq_s(1) <= pb0_rise_s OR pb0_fall_s;
        when "01" => intreq_s(1) <= pb0_rise_s;
        when "10" => intreq_s(1) <= pb0_fall_s;
        when "11" => intreq_s(1) <= '0';
        when others => intreq_s(1) <= '0';
      end case;
    else
      -- Disabled interrupt
      intreq_s(1) <= '0';
    end if;
  end process;


  process( sysclk_s, rst_s )
  begin
    if rst_s='1' then
      INTRQ <= (others => '0');
    elsif rising_edge(sysclk_s) then
      if sfr_wen(5)='1' then
        if DEBUG_ENABLED then
          report "SFR INTRQ write, data " & hstr(std_logic_vector(sfr_wdata)) & " mask " & hstr(std_logic_vector(sfr_wmask));
        end if;
        INTRQ <= (INTRQ and not sfr_wmask) or (sfr_wdata AND sfr_wmask);
      end if;
      -- Set interrupt even if we cleared from SW
      l: for i in 0 to 7 loop
        if intreq_s(i)='1' then
          INTRQ(i) <= '1';
        end if;
      end loop;

    end if;
  end process;


  with to_integer(dec.ioaddr)
    select sfr_read <=
      "0000" & flags.O & flags.AC & flags.C & flags.Z when 0,
      SP_s        when 2,
      CLKMD_s     when 3,
      INTEN_s     when 4,
      INTRQ       when 5,
      T16M_s      when 6,
      MISC_s      when 8,
      TM2B_s      when 9,
      x"00"       when 10, -- EOSCR is write-only
      INTEGS_s    when 12,
      PADIER_s    when 13,
      PBDIER_s    when 14,
      PA_s        when 16,
      PAC_s       when 17,
      PAPH_s      when 18,
      PB_s        when 20,
      PBC_s       when 21,
      PBPH_s      when 22,
      TM2S_s      when 23,
      GPCC        when 24,
      GPCS        when 25,
      TM2C_s      when 28,
      TM2CT_s     when 29,
      PWMG0C_s    when 32,
      PWMG0S_s    when 33,
      x"00"       when 34, -- PWMG0DTH is write-only
      x"00"       when 35, -- PWMG0DTL is write-only
      x"00"       when 36, -- PWMG0CUBH is write-only
      x"00"       when 37, -- PWMG0CUBL is write-only
      PWMG1C      when 38,
      PWMG1S      when 39,
      PWMG1DTH    when 40,
      PWMG1DTL    when 41,
      PWMG1CUBH   when 42,
      PWMG1CUBL   when 43,
      PWMG2C      when 44,
      PWMG2S      when 45,
      PWMG2DTH    when 46,
      PWMG2DTL    when 47,
      PWMG2CUBH   when 48,
      PWMG2CUBL   when 49,
      TM3C_s      when 50,
      TM3CT_s     when 51,
      TM3S_s      when 52,
      TM3B_s      when 53,
      (others => 'X') when others;

  -- ROM

  rom: entity work.pdkrom
  port map (
    PC_i => to_integer(pc),
    dat_o => opcode_full_s
  );


  -- Instruction fetcher
  process(sysclk_s,rst_s)
  begin
    if rst_s='1' then
       opcode_valid <= '0';
       PC <= (others => '0');
    elsif rising_edge(sysclk_s) then
      if stall_s='0' then
        if jmp='1' then
          PC <= npc;
          IPC <= npc; -- To avoid interrupt not picking up return address.
          opcode_valid <= '0';
          if DEBUG_ENABLED then
            report "Jumping to " & hstr(std_logic_vector(npc));
          end if;
        else 
          opcode_s <= opcode_full_s(opcode_s'HIGH downto 0);
          opcode_valid <= '1';
          PC <= PC + 1;
          IPC <= PC;
        end if;
      end if;
    end if;
  end process;

  -- Instruction decoder
  decoder_i0: entity work.pdk14decode
  port map (
    opcode_i    => opcode_s,
    valid_i     => opcode_valid,
    decoded_o   => dec
  );

  -- interrupt generation
  intsrc_s  <= std_logic_vector(INTEN_s) and std_logic_vector(INTRQ);
  int_s     <=  or_reduce(intsrc_s);


  -- Jump decoder
  process(dec.decoded, A, IPC, dec.jmpaddr, dec.memaddr, int_s, ginten, sfr_read)
    variable temp: unsigned(7 downto 0);
  begin
    jmp <= '0';
    if int_s='1' and ginten='1' then
      npc <= "000" & x"10";
      jmp <= '1';
    else
      case dec.decoded is
        when opcode_pcadda =>
          npc <= IPC + signextend11(A);
          jmp <= '1';
        when opcode_ret | opcode_reti | opcode_retk =>
          temp := readMem(to_integer(SP_s-1));
          npc(npc'HIGH downto 8) <= temp(2 downto 0);
          temp := readMem(to_integer(SP_s-2));
          npc(7 downto 0) <= temp;
          jmp <= '1';
        when opcode_gotok | opcode_callk =>
          npc <= dec.jmpaddr;
          jmp <= '1';
        when opcode_ceqsnam =>  
          if A = readMem( to_integer(dec.memaddr) ) then
            npc <= IPC + 2;
            jmp <= '1';
          end if;
        when opcode_cneqsnam =>  
          if A /= readMem( to_integer(dec.memaddr) ) then
            npc <= IPC + 2;
            jmp <= '1';
          end if;
        when opcode_ceqsnak =>  
          if A = dec.immed then
            npc <= IPC + 2;
            jmp <= '1';
          end if;
        when opcode_cneqsnak =>  
          if A /= dec.immed then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_t0snio =>
          if sfr_read( to_integer(dec.bitaddr) )='0' then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_t0snm =>
          if readMem( to_integer(dec.memaddr) )( to_integer(dec.bitaddr) )='0' then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_t1snio =>
          if sfr_read( to_integer(dec.bitaddr) )='1' then
            npc <= IPC + 2;
            jmp <= '1';
          end if;
          
        when opcode_t1snm =>
          if readMem( to_integer(dec.memaddr) )( to_integer(dec.bitaddr) )='1' then
            npc <= IPC + 2;
            jmp <= '1';
          end if;
  
        when others => 
      end case;
    end if;
  end process;


  -- SFR write
  process(dec.decoded, sfr_read, dec.ioaddr, dec.bitaddr, opcode_valid)
  begin
    sfr_wen <= (others => '0');
    sfr_wmask <= (others => '1');
    
    if int_s='1' and ginten='1' then
      sfr_wen(SFR_INDEX_SP)<='1';
      sfr_wdata <= SP_s + 2;
    else
      case dec.decoded is 
        when opcode_pushaf  =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s + 2;

        when opcode_popaf   =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s - 2;

        when opcode_ret     =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s - 2;

        when opcode_reti    =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s - 2;

        when opcode_retk    =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s - 2;
 
        when opcode_xorioa  =>  sfr_wdata <= A xor sfr_read;
                                sfr_wen( to_integer(dec.ioaddr) ) <= '1';

        when opcode_movioa  =>  sfr_wdata <= A;
                                sfr_wen( to_integer(dec.ioaddr) ) <= '1';
 
        when opcode_set0io  =>  sfr_wen( to_integer(dec.ioaddr) ) <= '1';
                                sfr_wmask <= (others => '0');
                                sfr_wmask(to_integer(dec.bitaddr)) <= '1';
                                sfr_wdata <= (others => '0');

        when opcode_set1io  =>  sfr_wen( to_integer(dec.ioaddr) ) <= '1';
                                sfr_wmask <= (others => '0');
                                sfr_wmask(to_integer(dec.bitaddr)) <= '1';
                                sfr_wdata <= (others => '1');

        when opcode_swapcio =>  sfr_wen( to_integer(dec.ioaddr) ) <= '1';
                                sfr_wmask(to_integer(dec.bitaddr)) <= '1';
                                sfr_wdata <= (others => flags.C);

        when opcode_callk   =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s + 2;

        when others =>
      end case;
    end if;
  end process;


  -- Main execution unit.

  process(sysclk_s, rst_s)
    variable zero: wordtype := (others => '0');
    variable temp: wordtype;
    variable temp16: unsigned(15 downto 0);
    variable temp7: ioaddrtype;
    variable flagstemp: flags_type;
    variable ipcn: pctype;
    variable opstr: string(1 to 7);

    impure function perform_add(b: in wordtype;c: in std_logic) return wordtype 
    is
      variable a_v, b_v, c_v, add_v: unsigned(8 downto 0);
      variable hadd_v: unsigned(4 downto 0);
      variable oadd_v: unsigned(7 downto 0);
      variable r_v: flags_type;
    begin
      a_v := "0" & A;
      b_v := "0" & b;
      c_v := x"00" & c;
      add_v := a_v + b_v + c_v;
      hadd_v := ("0" &a_v(3 downto 0)) + ("0" & b_v(3 downto 0)) + c_v(4 downto 0);
      oadd_v := ("0" &a_v(6 downto 0)) + ("0" & b_v(6 downto 0)) + c_v(7 downto 0);
      r_v.Z :='0';
      if (add_v(7 downto 0)=x"00") then
        r_v.Z := '1';
      end if;   
      r_v.C  := add_v(8);
      r_v.AC := hadd_v(4);
      r_v.O  := oadd_v(7) xor add_v(8);
      flags   <= r_v;
      return add_v(7 downto 0);
    end function;

    impure function perform_sub( b: in wordtype;
                          c: in std_logic) return wordtype
    is
      variable a_v, b_v, c_v, sub_v: signed(8 downto 0);
      variable ha_v: signed(4 downto 0);
      variable hb_v: signed(4 downto 0);
  
      variable osub_v: signed(7 downto 0);
      variable r_v: flags_type;
    begin
      a_v := '0' & signed(A);
      b_v := '0' & signed(b);
      c_v := x"00" & c;
      sub_v := a_v - b_v - c_v;
  
      r_v.Z :='0';
      if (sub_v(7 downto 0)=x"00") then
        r_v.Z := '1';
      end if;   
        
      r_v.C  := sub_v(8);
      
      ha_v := '0' & signed(a(3 downto 0));
      hb_v := '0' & signed(a(3 downto 0));
      hb_v := hb_v + c_v(4 downto 0);
      if (ha_v < hb_v) then
        r_v.AC := '1';
      else
        r_v.AC := '0';
      end if;
      -- TBD: check if carry is used below.
      osub_v := ("0" &a_v(6 downto 0)) - ("0" & b_v(6 downto 0)) - c_v(7 downto 0);
      r_v.O  := osub_v(7) xor sub_v(8);
      
      flags <= r_v;
      return unsigned(sub_v(7 downto 0));
    end function;

    procedure set_z(data: in unsigned(7 downto 0)) is
    begin
      if data=x"00" then flags.Z <= '1'; else flags.Z <= '0'; end if;
    end procedure;

    procedure assign_and_set_z(data: in unsigned(7 downto 0)) is
      variable temp: unsigned(7 downto 0);
    begin
      temp  := data;
      A <= temp;
      set_z(data);
    end procedure;

    variable t16edge: std_logic;
    variable t16intindex : natural;
    variable unimpl: boolean;

  begin
    
    if rst_s='1' then
      stall_s <= '0';
    elsif rising_edge(sysclk_s) then
    
      cycle <= cycle+1;
      if DEBUG_ENABLED then
        report hstr(std_logic_vector(cycle))&" PC:0x"&hstr(std_logic_vector(IPC)) 
          & " SP:0x" & hstr(std_logic_vector(SP_s))
          & " fl:[" & flagsname(flags) & "] "
          & " A:0x" & hstr(std_logic_vector(A)) 
          & " opcode " & str(opcode_s) & " " & opname(dec);
      end if;

      unimpl := false;
      if stall_s='1' then
        stall_s<='0';
      else
        if int_s='1' and ginten='1' then
          ginten<='0';
          ipcn := ipc;
          writeMem(to_integer(SP_s), ipcn(7 downto 0));
          writeMem( to_integer(SP_s+1) , "00000" & ipcn(10 downto 8));
          --addSP(2);
        else
        case dec.decoded is
          when opcode_nop     =>
          when opcode_addca   =>  A <= perform_add( x"00", flags.C );
          when opcode_subca   =>  A <= perform_sub( x"00", flags.C );
          when opcode_izsna   =>  A <= perform_add( x"01", '0' );
          when opcode_dzsna   =>  A <= perform_sub( x"01", '0' );
          when opcode_pcadda  =>  unimpl := true;
          when opcode_nota    =>  assign_and_set_z( not A );
          when opcode_nega    =>  assign_and_set_z( neg(A) );
          when opcode_sra     =>  temp := '0' & A(7 downto 1); A <= temp; flags.C <= A(0);
          when opcode_sla     =>  temp := A(6 downto 0) & '0'; A <= temp; flags.C <= A(7);
          when opcode_srca    =>  temp := flags.C & A(7 downto 1); A <= temp; flags.C <= A(0);
          when opcode_slca    =>  temp := A(6 downto 0) & flags.C; A <= temp; flags.C <= A(7);
          when opcode_swapa   =>  A(7 downto 4) <= A(3 downto 0); A(3 downto 0) <= A(7 downto 4);
          when opcode_wdreset =>  unimpl := true;
          when opcode_pushaf  =>  writeMem( to_integer(SP_s) , A);
                                  writeMem( to_integer(SP_s)+1 , "0000" & flags.O & flags.AC & flags.C & flags.Z );
                                  --addSP(2);
  
          when opcode_popaf   =>  A         <= readMem( to_integer(SP_s-2) );
                                  temp       := readMem( to_integer(SP_s-1) );
                                  --subSP(2);
                                  flags.Z   <= temp(0);
                                  flags.C   <= temp(1);
                                  flags.AC   <= temp(2);
                                  flags.O   <= temp(3);
  
          when opcode_reset   =>  unimpl := true;
          when opcode_stopsys =>  unimpl := true;
          when opcode_stopexe =>  unimpl := true;
          when opcode_engint  =>  ginten <= '1';
          when opcode_disgint =>  ginten <= '0';
          when opcode_ret     =>  null;
          when opcode_reti    =>  ginten <= '1';
          when opcode_mul     =>  unimpl := true;
          when opcode_xorioa  =>  null;
          when opcode_movioa  =>  null;
          when opcode_movaio  =>  assign_and_set_z( sfr_read );
          when opcode_retk    =>  A <= dec.immed; 
                                  --subSP(2);
          when opcode_ldtablm =>  unimpl := true;
          when opcode_ldtabhm =>  unimpl := true;
          when opcode_stt16m  =>  temp16(15 downto 8) := readMem( to_integer(dec.memaddr) );
                                  temp16(7 downto 0) := readMem( to_integer(dec.memaddr) + 1 );
                                  tim16setvalue <= temp16;
                                  tim16set := '1';

          when opcode_ldt16m  =>  unimpl := true;
          when opcode_idxmma  =>  temp := readMem( to_integer( dec.memaddr ) );
                                  writeMem( to_integer(temp), A );
                                  stall_s <= '1';

          when opcode_idxmam  =>  temp := readMem( to_integer( dec.memaddr ) );
                                  A <= readMem( to_integer(temp) );
                                  stall_s <= '1';

          when opcode_nmovam  =>  unimpl := true;
          when opcode_nmovma  =>  unimpl := true;
          when opcode_swapm   =>  unimpl := true;
          when opcode_compam  =>  unimpl := true;
          when opcode_compma  =>  unimpl := true;
          when opcode_naddam  =>  unimpl := true;
          when opcode_naddma  =>  unimpl := true;
          when opcode_addma   =>  unimpl := true;
          when opcode_subma   =>  unimpl := true;
          when opcode_addcma  =>  unimpl := true;
          when opcode_subcma  =>  unimpl := true;
          when opcode_andma   =>  temp := readMem(to_integer(dec.memaddr));
                                  temp := temp AND A;
                                  writeMem(to_integer(dec.memaddr),temp);
                                 set_z(temp);


          when opcode_orma    => temp := readMem(to_integer(dec.memaddr));
                                 temp := temp OR A;
                                 writeMem(to_integer(dec.memaddr),temp);
                                 set_z(temp);

          when opcode_xorma   => unimpl := true;
          when opcode_movma   => writeMem( to_integer(dec.memaddr), A);
          when opcode_addam   =>  temp :=  readMem(to_integer(dec.memaddr));
                                  A <= perform_add( temp, '0' );


          --unimpl := true;
          when opcode_subam   => unimpl := true;
          when opcode_addcam  => unimpl := true;
          when opcode_subcam  => unimpl := true;
          when opcode_andam   => unimpl := true;
          when opcode_oram    => unimpl := true;
          when opcode_xoram   => unimpl := true;
          when opcode_movam   => A <= readMem( to_integer(dec.memaddr) );
          when opcode_addcm   => unimpl := true;
          when opcode_subcm   => unimpl := true;
          when opcode_izsnm   => unimpl := true;
          when opcode_dzsnm   => unimpl := true;
          when opcode_incm    =>  temp := readMem( to_integer(dec.memaddr) );
                                  writeMem( to_integer(dec.memaddr), temp + 1);

                                  --unimpl := true;
          when opcode_decm    => unimpl := true;
          when opcode_clearm  => writeMem( to_integer(dec.memaddr), (others => '0'));
          when opcode_xchm    =>  unimpl := true;
          when opcode_notm    =>  unimpl := true;
          when opcode_negm    =>  unimpl := true;
          when opcode_srm     =>  temp := readMem(to_integer(dec.memaddr));
                                  flags.C <= temp(0);
                                  writeMem( to_integer(dec.memaddr), '0' & temp(7 downto 1));

          when opcode_slm     =>  temp := readMem(to_integer(dec.memaddr));
                                  flags.C <= temp(7);
                                  writeMem( to_integer(dec.memaddr), temp(6 downto 0) & '0');

          when opcode_srcm    =>  temp := readMem(to_integer(dec.memaddr));
                                  flags.C <= temp(0);
                                  writeMem( to_integer(dec.memaddr), flags.C & temp(7 downto 1));

          when opcode_slcm    =>  temp := readMem(to_integer(dec.memaddr));
                                  flags.C <= temp(7);
                                  writeMem( to_integer(dec.memaddr), temp(6 downto 0) & flags.C);

          when opcode_ceqsnam =>  temp := perform_sub(readMem(to_integer(dec.memaddr)),'0');
          when opcode_cneqsnam=>  temp := perform_sub(readMem(to_integer(dec.memaddr)),'0');
          when opcode_t0snio  =>  null;
          when opcode_t1snio  =>  null;
          when opcode_set0io  =>  null;
          when opcode_set1io  =>  null;
          when opcode_t0snm   =>  null;
          when opcode_t1snm   =>  null;
          when opcode_set0m   =>  temp := readMem( to_integer(dec.memaddr) );
                                  temp( to_integer( dec.bitaddr ) ) := '0';
                                  writeMem( to_integer(dec.memaddr), temp );
          when opcode_set1m   =>  temp := readMem( to_integer(dec.memaddr) );
                                  temp( to_integer( dec.bitaddr ) ) := '1';
                                  writeMem( to_integer(dec.memaddr), temp );
          when opcode_addak   =>  A <= perform_add(dec.immed, '0');
          when opcode_subak   =>  A <= perform_sub(dec.immed, '0');
          when opcode_ceqsnak =>  temp := perform_sub( dec.immed, '0');
          when opcode_cneqsnak=>  temp := perform_sub( dec.immed, '0');
          when opcode_andak   =>  assign_and_set_z( A and dec.immed );
          when opcode_orak    =>  assign_and_set_z( A or dec.immed );
          when opcode_xorak   =>  assign_and_set_z( A xor dec.immed );
          when opcode_movak   =>  A <= dec.immed;
          when opcode_swapcio =>  unimpl := true;
          when opcode_gotok   =>  null;
          when opcode_callk   =>  ipcn := ipc + 1;
                                  writeMem(to_integer(SP_s), ipcn(7 downto 0));
                                  writeMem( to_integer(SP_s+1) , "00000" & ipcn(10 downto 8));
                                  --addSP(2);
          
          when others         =>  unimpl := true;
        end case;
        end if;
      
        if unimpl then
          report "UNIMPLEMENTED" severity failure;
        end if;
      end if; -- stall


      -- TBD: this should be moved to its own module once we get
      -- dualport registers.

      -- Generate T16 interrupt
      pt16cnt <= t16cnt;
      t16edge := INTEGS_s(4);
      intreq_s(2) <= '0';
      t16intindex := to_integer (T16M_s(2 downto 0) ) + 8;
      --report "INT INDEX " & str(t16intindex);
      if (pt16cnt(t16intindex)=t16edge and t16cnt(t16intindex)=not t16edge) then
        --report "Interrupt";
        --SFR_INTRQ(2) <= '1';
        intreq_s(2) <= '1';
      end if;
      
    end if;

  end process;


  -- TBD: this should be moved to its own module once we get
  -- dualport registers.
  tim16b: block
    signal pres: unsigned(5 downto 0);
    signal en: std_logic;
  begin

    with T16M_s(4 downto 3) select en <=
      '1' when "00",
      pres(1) and not pres(0) when "01",
      pres(3) and not pres(2) and not pres(1) and not pres(0) when "10",
      pres(5) and not pres(4) and not pres(3) and not pres(2) and not pres(1) and not pres(0)when "11",
      '0' when others;


    timer16clk <= sysclk_s;
    
    timer16: process(timer16clk, rst_s)
    begin
      if rst_s='1' then
        pres <= (others => '0');
        t16cnt <=(others => '0');
      elsif rising_edge(timer16clk) then
        if tim16set='1' then
          tim16set:='0';
          --report "Rset timer" severity failure;
          t16cnt <= tim16setvalue;
        end if;
        
        
        pres <= pres + 1;
        --report "pres " & str(std_logic_vector(pres));
        if en='1' then
          t16cnt <= t16cnt + 1;
          --report "tim16 " & hstr(std_logic_vector(t16cnt));
        end if;

        --report "CLKSEL " & str(std_logic_vector(sfr(6)(7 downto 0)));
      end if;
    end process;
    
    
  end block;

  -- PA Inputs directly from pads
  PA_i_s <= PA_io;

  -- PA outputs
  PA_io(0) <= PA_o_s(0);
  PA_io(1) <= PA_o_s(1);
  PA_io(2) <= PA_o_s(2);
  PA_io(3) <= tim2out_s when tm2c_s(3 downto 2)="10" else PA_o_s(3);
  PA_io(4) <= PA_o_s(4);
  PA_io(5) <= PA_o_s(5);
  PA_io(6) <= PA_o_s(6) when not EOSC_CONNECTED else 'X';
  PA_io(7) <= PA_o_s(7) when not EOSC_CONNECTED else 'X';

  -- PB Inputs directly from pads
  PB_i_s <= PB_io;

  -- PB outputs
  PB_io(0) <= PB_o_s(0);
  PB_io(1) <= PB_o_s(1);
  PB_io(2) <= tim2out_s when tm2c_s(3 downto 2)="01" else PB_o_s(2);
  PB_io(3) <= PB_o_s(3);
  PB_io(4) <= tim2out_s when tm2c_s(3 downto 2)="11" else PB_o_s(4);
  PB_io(5) <= tim3out_s when tm3c_s(3 downto 2)="01"
              else pwmg0out_s when pwmg0c_s(3 downto 1)="001"
              else PB_o_s(5);
  PB_io(6) <= tim3out_s when tm3c_s(3 downto 2)="10" else PB_o_s(6);
  PB_io(7) <= tim3out_s when tm3c_s(3 downto 2)="11" else PB_o_s(7);

end sim;
