--
--  FPPA PDK14 Microcontroller simulation model - Decoder block
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
use work.pdk14pkg.all;

entity pdk14decode is 
	port (
  	opcode_i	:	in opcodetype;
    valid_i		: in std_logic;
    decoded_o : out opdec_type
  );
end entity;
    
architecture sim of pdk14decode is

	signal dec: opdec_type;
  
  
begin


	process(opcode_i, valid_i)
  	variable dt: std_logic_vector(1 downto 0);
  begin

		dec.immed 	<= unsigned(opcode_i(7 downto 0));
    dec.ioaddr	<= unsigned(opcode_i(5 downto 0));
    dec.memaddr	<= unsigned(opcode_i(6 downto 0));
    dec.jmpaddr <= unsigned(opcode_i(10 downto 0));
    dec.bitaddr <= unsigned(opcode_i(8 downto 6));
    if valid_i='1' then
      if opcode_i(13 downto 5)="000000000" then
        dec.decoded <= opcode_nop;       
      elsif opcode_i(13 downto 5)="000000011" then
        case opcode_i(4 downto 0) is
          when "00000" => dec.decoded <= opcode_addca;
          when "00001" => dec.decoded <= opcode_subca;
          when "00010" => dec.decoded <= opcode_izsna;
          when "00011" => dec.decoded <= opcode_dzsna;
          when "00111" => dec.decoded <= opcode_pcadda;
          when "01000" => dec.decoded <= opcode_nota;
          when "01001" => dec.decoded <= opcode_nega;
          when "01010" => dec.decoded <= opcode_sra;
          when "01011" => dec.decoded <= opcode_sla;
          when "01100" => dec.decoded <= opcode_srca;
          when "01101" => dec.decoded <= opcode_slca;
          when "01110" => dec.decoded <= opcode_swapa;
          when "10000" => dec.decoded <= opcode_wdreset;
          when "10010" => dec.decoded <= opcode_pushaf;
          when "10011" => dec.decoded <= opcode_popaf;
          when "10101" => dec.decoded <= opcode_reset;
          when "10110" => dec.decoded <= opcode_stopsys;
          when "10111" => dec.decoded <= opcode_stopexe;
          when "11000" => dec.decoded <= opcode_engint;
          when "11001" => dec.decoded <= opcode_disgint;
          when "11010" => dec.decoded <= opcode_ret;
          when "11011" => dec.decoded <= opcode_reti;
          when "11100" => dec.decoded <= opcode_mul;
          when others  => dec.decoded <= opcode_unknown;
        end case;
      elsif opcode_i(13 downto 9)="00000" then
        case opcode_i(8 downto 6) is
          when "011" 	 => dec.decoded <= opcode_xorioa;
          when "110" 	 => dec.decoded <= opcode_movioa;
          when "111" 	 => dec.decoded <= opcode_movaio;
          when others  => dec.decoded <= opcode_unknown;
        end case;
        -- IO
      elsif opcode_i(13 downto 8)="000010" then
        dec.decoded <= opcode_retk;
                        --000011 10000000
      elsif opcode_i(13 downto 8)="000011" then
      	dt := opcode_i(8) & opcode_i(0);
        dec.memaddr <= unsigned(opcode_i(6 downto 1) & '0'); -- TBD: check this please
        case dt is
          when "00"   => dec.decoded <= opcode_stt16m;
          when "01"   => dec.decoded <= opcode_ldt16m;
          when "10"   => dec.decoded <= opcode_idxmma;
          when "11"   => dec.decoded <= opcode_idxmam;
          when others  => dec.decoded <= opcode_unknown;
        end case;
      elsif opcode_i(13 downto 9)="00010" then
        dec.decoded <= opcode_swapcio;
        
      elsif opcode_i(13 downto 12)="00" then
        case opcode_i(11 downto 7) is
                --when "01000" => dec.decoded <= opcode_nmovam;
                --when "01001" => dec.decoded <= opcode_nmovma;
                --when "01010" => dec.decoded <= opcode_swapm;
          when "01100" => dec.decoded <= opcode_compam;
          when "01101" => dec.decoded <= opcode_compma;
          when "01110" => dec.decoded <= opcode_naddam;
          when "01111" => dec.decoded <= opcode_naddma;
          when "10000" => dec.decoded <= opcode_addma;
          when "10001" => dec.decoded <= opcode_subma;
          when "10010" => dec.decoded <= opcode_addcma;
          when "10011" => dec.decoded <= opcode_subcma;
          when "10100" => dec.decoded <= opcode_andma;
          when "10101" => dec.decoded <= opcode_orma;
          when "10110" => dec.decoded <= opcode_xorma;
          when "10111" => dec.decoded <= opcode_movma;
          when "11000" => dec.decoded <= opcode_addam;
          when "11001" => dec.decoded <= opcode_subam;
          when "11010" => dec.decoded <= opcode_addcam;
          when "11011" => dec.decoded <= opcode_subcam;
          when "11100" => dec.decoded <= opcode_andam;
          when "11101" => dec.decoded <= opcode_oram;
          when "11110" => dec.decoded <= opcode_xoram;
          when "11111" => dec.decoded <= opcode_movam;
          when others  => dec.decoded <= opcode_unknown;
        end case;
      elsif opcode_i(13 downto 11)="010" then
        case opcode_i(10 downto 7) is
          when "0000" => dec.decoded <= opcode_addcm;
          when "0001" => dec.decoded <= opcode_subcm;
          when "0010" => dec.decoded <= opcode_izsnm;
          when "0011" => dec.decoded <= opcode_dzsnm;
          when "0100" => dec.decoded <= opcode_incm;
          when "0101" => dec.decoded <= opcode_decm;
          when "0110" => dec.decoded <= opcode_clearm;
          when "0111" => dec.decoded <= opcode_xchm;
          when "1000" => dec.decoded <= opcode_notm;
          when "1001" => dec.decoded <= opcode_negm;
          when "1010" => dec.decoded <= opcode_srm;
          when "1011" => dec.decoded <= opcode_slm;
          when "1100" => dec.decoded <= opcode_srcm;
          when "1101" => dec.decoded <= opcode_slcm;
          when "1110" => dec.decoded <= opcode_ceqsnam;
          when "1111" => dec.decoded <= opcode_cneqsnam;
          when others => dec.decoded <= opcode_unknown;
        end case;
      elsif opcode_i(13 downto 11)="011" then
        --dec.ioaddr(7) <= '0';
        case opcode_i(10 downto 9) is
        	when "00"   => dec.decoded <= opcode_t0snio;
          when "01"   => dec.decoded <= opcode_t1snio;
          when "10"   => dec.decoded <= opcode_set0io;
          when "11"   => dec.decoded <= opcode_set1io;
          when others => dec.decoded <= opcode_unknown;
        end case;
      elsif opcode_i(13 downto 11)="100" then
        dec.memaddr(6) <= '0';
        case opcode_i(10 downto 9) is
          when "00"   => dec.decoded <= opcode_t0snm;
          when "01"   => dec.decoded <= opcode_t1snm;
          when "10"   => dec.decoded <= opcode_set0m;
          when "11"   => dec.decoded <= opcode_set1m;
          when others => dec.decoded <= opcode_unknown;
        end case;

      elsif opcode_i(13 downto 11)="101" then
        case opcode_i(10 downto 8) is
        	when "000"  => dec.decoded <= opcode_addak;
          when "001"  => dec.decoded <= opcode_subak;
          when "010"  => dec.decoded <= opcode_ceqsnak;
          when "011"  => dec.decoded <= opcode_cneqsnak;
          when "100"  => dec.decoded <= opcode_andak;
          when "101"  => dec.decoded <= opcode_orak;
          when "110"  => dec.decoded <= opcode_xorak;
          when "111"  => dec.decoded <= opcode_movak;
          when others => dec.decoded <= opcode_unknown;
        end case;

      elsif opcode_i(13 downto 12)="11" then
        case opcode_i(11) is
          when '0' 		=> dec.decoded <= opcode_gotok;
          when '1' 		=> dec.decoded <= opcode_callk;
          when others => dec.decoded <= opcode_unknown;
        end case;
      else
    	  dec.decoded <= opcode_unknown;
      end if;
    else
    	dec.decoded <= opcode_nop;
  	end if;
  end process;


	decoded_o <= dec;
  
end sim;
