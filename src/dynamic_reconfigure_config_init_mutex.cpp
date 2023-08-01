#include <dynamic_reconfigure/config_init_mutex.h>

boost::mutex dynamic_reconfigure::__init_mutex__;
bool dynamic_reconfigure::__migrate_param__;
