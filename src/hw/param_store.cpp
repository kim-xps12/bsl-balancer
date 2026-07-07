#include "param_store.h"

#include <Preferences.h>

namespace hw {
namespace {
constexpr const char* kNs = "bslbal";
constexpr const char* kKeyParams = "tparams";
}  // namespace

bool ParamStore::loadParams(core::TuningParams* out) {
  Preferences prefs;
  if (!prefs.begin(kNs, /*readOnly=*/true)) return false;
  core::TuningParams rec;
  const size_t n = prefs.getBytes(kKeyParams, &rec, sizeof(rec));
  prefs.end();
  if (n != sizeof(rec)) return false;
  if (!core::validateParams(rec)) return false;  // fail-closed: 既定値のまま
  *out = rec;
  return true;
}

bool ParamStore::saveParams(const core::TuningParams& rec) {
  if (!core::validateParams(rec)) return false;
  Preferences prefs;
  if (!prefs.begin(kNs, /*readOnly=*/false)) return false;
  const size_t n = prefs.putBytes(kKeyParams, &rec, sizeof(rec));
  prefs.end();
  return n == sizeof(rec);
}

}  // namespace hw
