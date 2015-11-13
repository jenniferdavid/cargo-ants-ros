#include "smart_params.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/util/OptionDictionary.hpp>
#include <cmath>

using namespace sfl;
using namespace boost;

smartparams::
smartparams(shared_ptr<sfl::OptionDictionary> opt)
  : expo_parameters(opt),
    model_axlewidth(0.8),
    model_phi_max(1.0),
    model_phid_max(3.0)
{
  front_channel     = 0;
  front_nscans      = 181;
  front_mount_x     = 0.15;
  front_mount_y     = 0;
  front_mount_theta = 0;
  front_rhomax      = 8;
  front_phi0        = -M_PI/2;
  front_phirange    = M_PI;
  
  rear_channel      = 1;
  rear_nscans       = 181;
  rear_mount_x      = -0.15;
  rear_mount_y      = 0;
  rear_mount_theta  = M_PI;
  rear_rhomax       = 8;
  rear_phi0         = -M_PI/2;
  rear_phirange     = M_PI;
  
  string_to(opt->GetOption("model_axlewidth"), model_axlewidth);
  string_to(opt->GetOption("model_phi_max"), model_phi_max);
  string_to(opt->GetOption("model_phid_max"), model_phid_max);

  string_to(opt->GetOption("front_channel"), front_channel);
  string_to(opt->GetOption("front_nscans"), front_nscans);
  string_to(opt->GetOption("front_mount_x"), front_mount_x);
  string_to(opt->GetOption("front_mount_y"), front_mount_y);
  string_to(opt->GetOption("front_mount_theta"), front_mount_theta);
  string_to(opt->GetOption("front_rhomax"), front_rhomax);
  string_to(opt->GetOption("front_phi0"), front_phi0);
  string_to(opt->GetOption("front_phirange"), front_phirange);
  
  string_to(opt->GetOption("rear_channel"), rear_channel);
  string_to(opt->GetOption("rear_nscans"), rear_nscans);
  string_to(opt->GetOption("rear_mount_x"), rear_mount_x);
  string_to(opt->GetOption("rear_mount_y"), rear_mount_y);
  string_to(opt->GetOption("rear_mount_theta"), rear_mount_theta);
  string_to(opt->GetOption("rear_rhomax"), rear_rhomax);
  string_to(opt->GetOption("rear_phi0"), rear_phi0);
  string_to(opt->GetOption("rear_phirange"), rear_phirange);
}
