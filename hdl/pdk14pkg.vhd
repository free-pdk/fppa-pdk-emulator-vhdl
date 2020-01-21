--
--  FPPA PDK14 Microcontroller simulation model - PDK14 package
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
use work.txt_util.all;

package pdk14pkg is

  subtype wordtype is unsigned(7 downto 0);

  -- SYM_85A
  subtype opcodetype is std_logic_vector(13 downto 0);
  subtype pctype is unsigned(10 downto 0); -- 2048 words
  subtype ioaddrtype is unsigned(5 downto 0);
  subtype memaddrtype is unsigned(6 downto 0);

  type decoded_opcode_type is (
    opcode_unknown,
    opcode_nop,
    opcode_addca,
    opcode_subca,
    opcode_izsna,
    opcode_dzsna,
    opcode_pcadda,
    opcode_nota,
    opcode_nega,
    opcode_sra,
    opcode_sla,
    opcode_srca,
    opcode_slca,
    opcode_swapa,
    opcode_wdreset,
    opcode_pushaf,
    opcode_popaf,
    opcode_reset,
    opcode_stopsys,
    opcode_stopexe,
    opcode_engint,
    opcode_disgint,
    opcode_ret,
    opcode_reti,
    opcode_mul,
    opcode_xorioa,
    opcode_movioa,
    opcode_movaio,
    opcode_retk,
    opcode_ldtablm,
    opcode_ldtabhm,
    opcode_stt16m,
    opcode_ldt16m,
    opcode_idxmma,
    opcode_idxmam,
    opcode_nmovam,
    opcode_nmovma,
    opcode_swapm,
    opcode_compam,
    opcode_compma,
    opcode_naddam,
    opcode_naddma,
    opcode_addma,
    opcode_subma,
    opcode_addcma,
    opcode_subcma,
    opcode_andma,
    opcode_orma,
    opcode_xorma,
    opcode_movma,
    opcode_addam,
    opcode_subam,
    opcode_addcam,
    opcode_subcam,
    opcode_andam,
    opcode_oram,
    opcode_xoram,
    opcode_movam,
    opcode_addcm,
    opcode_subcm,
    opcode_izsnm,
    opcode_dzsnm,
    opcode_incm,
    opcode_decm,
    opcode_clearm,
    opcode_xchm,
    opcode_notm,
    opcode_negm,
    opcode_srm,
    opcode_slm,
    opcode_srcm,
    opcode_slcm,
    opcode_ceqsnam,
    opcode_cneqsnam,
    opcode_t0snio,
    opcode_t1snio,
    opcode_set0io,
    opcode_set1io,
    opcode_t0snm,
    opcode_t1snm,
    opcode_set0m,
    opcode_set1m,
    opcode_addak,
    opcode_subak,
    opcode_ceqsnak,
    opcode_cneqsnak,
    opcode_andak,
    opcode_orak,
    opcode_xorak,
    opcode_movak,
    opcode_swapcio,
    opcode_gotok,
    opcode_callk
  );

  type opdec_type is record
    immed 		: wordtype;
    ioaddr    : ioaddrtype;
    memaddr   : memaddrtype;
    bitaddr		: unsigned(2 downto 0);
    jmpaddr   : pctype;
    decoded  	: decoded_opcode_type;
  end record;

  type flags_type is record
    C: std_logic;
    Z: std_logic;
    O: std_logic;
    AC:std_logic;
  end record;


  function opname(op: in opdec_type) return string;
  function flagsname(f: in flags_type) return string;

end package;

package body pdk14pkg is

  function flagsname(f: in flags_type) return string is
    variable r: string(1 to 4) := "    ";
  begin
    if f.C='1' then r(3):='C'; end if;
    if f.Z='1' then r(4):='Z'; end if;
    if f.AC='1' then r(2):='A'; end if;
    if f.O='1' then r(1):='O'; end if;
    return r;
  end function;

  function ioname(op: in opdec_type) return string is
  begin
    return "IO[0x" & hstr(std_logic_vector(op.ioaddr)) & "]";
  end function;

  function memname(op: in opdec_type) return string is
  begin
    return "[0x" & hstr(std_logic_vector(op.memaddr)) & "]";
  end function;

  function immedname(op: in opdec_type) return string is
  begin
    return "#0x" & hstr(std_logic_vector(op.immed));
  end function;

  function bitname(op: in opdec_type) return string is
  begin
    return str(to_integer(op.bitaddr));
  end function;

  function opname(op: in opdec_type) return string is
  begin
  	case op.decoded is
        when opcode_nop     => return "NOP     ";
        when opcode_addca   => return "ADDC A  ";
        when opcode_subca   => return "SUBC A  ";
        when opcode_izsna   => return "IZSN A  ";
        when opcode_dzsna   => return "DZSN A  ";
        when opcode_pcadda  => return "PCADD A ";
        when opcode_nota    => return "NOT A   ";
        when opcode_nega    => return "NEG A   ";
        when opcode_sra     => return "SR A    ";
        when opcode_sla     => return "SL A    ";
        when opcode_srca    => return "SRC A   ";
        when opcode_slca    => return "SLC A   ";
        when opcode_swapa   => return "SWAP A  ";
        when opcode_wdreset => return "WDRESET ";
        when opcode_pushaf  => return "PUSHAF  ";
        when opcode_popaf   => return "POPAF   ";
        when opcode_reset   => return "RESET   ";
        when opcode_stopsys => return "STOPSYS ";
        when opcode_stopexe => return "STOPEXE ";
        when opcode_engint  => return "ENGINT  ";
        when opcode_disgint => return "DISGINT ";
        when opcode_ret     => return "RET     ";
        when opcode_reti    => return "RETI    ";
        when opcode_mul     => return "MUL     ";
        when opcode_xorioa  => return "XOR "&ioname(op)&", A";
        when opcode_movioa  => return "MOV "&ioname(op)&", A";
        when opcode_movaio  => return "MOV A, "&ioname(op);
        when opcode_retk    => return "RET " & immedname(op);
        when opcode_ldtablm => return "LDTABL " & memname(op);
        when opcode_ldtabhm => return "LDTABH " & memname(op);
        when opcode_stt16m  => return "STT16 " & memname(op);
        when opcode_ldt16m  => return "LDT16 "  & memname(op);
        when opcode_idxmma  => return "IXDM " &memname(op) & ", A";
        when opcode_idxmam  => return "IDXM A, " & memname(op);
        when opcode_nmovam  => return "NMOV A, " & memname(op);
        when opcode_nmovma  => return "NMOV " & memname(op) & ", A";
        when opcode_swapm   => return "SWAP " & memname(op);
        when opcode_compam  => return "COMP A, " & memname(op) ;
        when opcode_compma  => return "COMP " & memname(op) & ", A";
        when opcode_naddam  => return "NADD A, "& memname(op);
        when opcode_naddma  => return "NADD "& memname(op) & ", A";
        when opcode_addma   => return "ADD "& memname(op) &", A";
        when opcode_subma   => return "SUB "& memname(op) &", A";
        when opcode_addcma 	=> return "ADDC "& memname(op) &",A";
        when opcode_subcma 	=> return "SUBC "& memname(op) &",A";
        when opcode_andma 	=> return "AND "& memname(op) &", A";
        when opcode_orma 		=> return "OR "& memname(op) &",A  ";
        when opcode_xorma 	=> return "XOR "& memname(op) &", A";
        when opcode_movma 	=> return "MOV "& memname(op) &", A";
        when opcode_addam 	=> return "ADD A, "& memname(op);
        when opcode_subam 	=> return "SUB A, "& memname(op);
        when opcode_addcam 	=> return "ADDC A, "& memname(op);
        when opcode_subcam 	=> return "SUBC A, "& memname(op);
        when opcode_andam 	=> return "AND A, "& memname(op);
        when opcode_oram 		=> return "OR A,"& memname(op);
        when opcode_xoram 	=> return "XOR A, "& memname(op);
        when opcode_movam 	=> return "MOV A, "& memname(op);
        when opcode_addcm 	=> return "ADDC "& memname(op);
        when opcode_subcm 	=> return "SUBC "& memname(op);
        when opcode_izsnm 	=> return "IZSN "& memname(op);
        when opcode_dzsnm 	=> return "DZSN "& memname(op);
        when opcode_incm 		=> return "INC "& memname(op);
        when opcode_decm 		=> return "DEC "& memname(op);
        when opcode_clearm  => return "CLEAR "& memname(op);
        when opcode_xchm 		=> return "XCH "& memname(op);
        when opcode_notm 		=> return "NOT "& memname(op);
        when opcode_negm 		=> return "NEG "& memname(op);
        when opcode_srm 		=> return "SR "& memname(op);
        when opcode_slm 		=> return "SL "& memname(op);
        when opcode_srcm 		=> return "SRC "& memname(op);
        when opcode_slcm 		=> return "SLC "& memname(op);
        when opcode_ceqsnam => return "CEQSN A, "& memname(op);
        when opcode_cneqsnam=> return "CNEQSN A, "& memname(op);
        when opcode_t0snio 	=> return "T0SN " & ioname(op) & "." & bitname(op);
        when opcode_t1snio 	=> return "T1SN "& ioname(op)& "." & bitname(op);
        when opcode_set0io 	=> return "SET0 "& ioname(op)& "." & bitname(op);
        when opcode_set1io 	=> return "SET1 "& ioname(op)& "." & bitname(op);
        when opcode_t0snm 	=> return "T0SN "& memname(op)& "." & bitname(op);
        when opcode_t1snm 	=> return "T1SN "& memname(op)& "." & bitname(op);
        when opcode_set0m 	=> return "SET0 "& memname(op)& "." & bitname(op);
        when opcode_set1m 	=> return "SET1 "& memname(op)& "." & bitname(op);
        when opcode_addak 	=> return "ADD A, "&immedname(op);
        when opcode_subak 	=> return "SUB A, "&immedname(op);
        when opcode_ceqsnak => return "CEQSN A,"&immedname(op);
        when opcode_cneqsnak=> return "CNEQSN A,"&immedname(op);
        when opcode_andak 	=> return "AND A, "&immedname(op);
        when opcode_orak  	=> return "OR A, "&immedname(op);
        when opcode_xorak 	=> return "XOR A, "&immedname(op);
        when opcode_movak 	=> return "MOV A, "&immedname(op);
        when opcode_swapcio => return "SWAPC " &ioname(op) & "." & bitname(op);
        when opcode_gotok 	=> return "GOTO "&immedname(op);
        when opcode_callk 	=> return "CALL "&immedname(op);
        when others =>         return "UNKNOWN ";
      end case;
  end function;


end;
