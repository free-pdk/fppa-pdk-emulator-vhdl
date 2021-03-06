library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity test is
end entity test;

architecture sim of test is

  signal pa_s :  std_logic_vector(7 downto 0);
  signal pb_s :  std_logic_vector(7 downto 0);

  constant DEBUG_ENABLED: boolean := false;

begin

  dut: entity work.pdk14
    generic map (
      DEBUG_ENABLED => DEBUG_ENABLED
    )
    port map (
      PA_io => pa_s,
      PB_io => pb_s
    );

  m1: entity work.measperiod
    generic map (
      NAME => "PA3"
    )
    port map (
      sig_i   => pa_s(3)
    );

  m2: entity work.measperiod
    generic map (
      NAME => "PB5"
    )
    port map (
      sig_i   => pb_s(5)
    );


  stim: process
  begin
    wait for 500 us;
    report "Ending simulation" severity failure;
  end process;

end sim;
