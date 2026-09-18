#!/bin/bash
set -e
REPO=/home/jschoi/edit_fastlio2
echo "##### 10_d_minimize sweep starting #####"
bash "$REPO/scripts/leafsweep_mid360_10d.sh"
echo "##### 10_d_minimize sweep done, starting 15_a #####"
bash "$REPO/scripts/leafsweep_mid360_15a.sh"
echo "##### 15_a sweep done, starting 15_c #####"
bash "$REPO/scripts/leafsweep_mid360_15c.sh"
echo "##### ALL THREE SWEEPS COMPLETE #####"
