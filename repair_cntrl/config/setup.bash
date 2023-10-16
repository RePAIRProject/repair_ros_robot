SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

export XBOT2_DEFAULT_HW=ec_idle
export ECAT_MASTER_CONFIG="$SCRIPT_DIR/ecat_master/ecat_config.yaml"
alias ecat_master="stdbuf --output=L --error=L repl -f $ECAT_MASTER_CONFIG 2>&1 | tee /tmp/ecat-output"
alias ecat_master_gdb="gdb --args repl -f $ECAT_MASTER_CONFIG"
