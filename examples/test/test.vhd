library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity test is
end entity test;

architecture sim of test is

  signal pa_s :  std_logic_vector(7 downto 0);
  signal pb_s :  std_logic_vector(7 downto 0);

begin

  dut: entity work.pdk14
    port map (
      PA_io => pa_s,
      PB_io => pb_s
    );


end sim;
