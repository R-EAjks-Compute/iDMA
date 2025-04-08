#!/bin/bash
cd ~/iDMA/
source venv/bin/activate

make idma_sim_all

cd target/sim/vsim

VSIM="questa-2023.4 vsim"

$VSIM -c -do "source compile.tcl; quit"

$VSIM -c -t 1ps -voptargs=+acc \
     +job_file=jobs/backend_rw_axi/simple.txt \
     -logfile rw_axi_simple.log \
     -wlf rw_axi_simple.wlf \
     tb_idma_backend_r_axi_r_obi_w_axi \
     -do "source start.tcl; run -all"

cd ~/iDMA/