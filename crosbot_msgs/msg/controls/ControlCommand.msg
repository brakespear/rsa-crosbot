Header      header

# Command messages may target a particular namespace. Empty namespace targets "all"
# (can't directly call this field namespace, due to C++ reserved keywords)
string      cmd_namespace

# Single string for command (found in controlCommands.hpp)
#    and set of parameters for the command
string      command
string[]    args

# Common Commands
string CMD_EM_STOP      = command_EM_STOP
string CMD_RESET        = command_reset
string CMD_RESUME       = command_resume
string CMD_START        = command_start
string CMD_STOP         = command_stop
