// Both of the bench's catalogs, and the choice between them.
//
// SIDs belong to a catalog. A table used against the wrong device addresses the wrong nodes and
// returns plausible nonsense, which is worse than returning nothing -- so the table is not a
// build flag. It is chosen from the checksum the device reports about itself, and until it has
// been asked the firmware is using a guess.
//
// This header exists so the selector is defined before anything that calls it. coreconf.h uses
// ketiSidFor throughout, and a header cannot call a function declared in a header included after
// it.
#pragma once

#include "sid_table.h"        // LAN9662, and the shared declarations
#include "sid_table_9692.h"   // LAN9692

int activeCatalog = 0;   // 0 = LAN9662, 1 = LAN9692

inline uint32_t ketiSidFor(const char *path) {
  return activeCatalog == 1 ? ketiSidFor9692(path) : ketiSidFor9662(path);
}

// False for a catalog this firmware has no table for -- worth refusing rather than guessing at.
inline bool selectCatalog(const String &checksum) {
  if (checksum == KETI_SID_CATALOG_CHECKSUM_9662) { activeCatalog = 0; return true; }
  if (checksum == KETI_SID_CATALOG_CHECKSUM_9692) { activeCatalog = 1; return true; }
  return false;
}
