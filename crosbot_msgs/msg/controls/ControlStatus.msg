
Header      header

# Status messages may come from a specific namespace. Empty namespace comes from "all".
# (can't directly call this field namespace, due to C++ reserved keywords)
string      stats_namespace

# Level of the status (info, error, warning)
string      level

# Status description
string      status
string[]    args

# Status values
string LEVEL_ERROR          = "level_error"
string LEVEL_INFO           = "level_info"
string LEVEL_WARNING        = "level_warning"
