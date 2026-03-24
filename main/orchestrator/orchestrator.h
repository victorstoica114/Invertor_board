#pragma once

#include "esp_err.h"

#include "orchestrator/protocol_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t orchestratorStart(protocol_id_t bmsProtocol, protocol_id_t inverterProtocol);

#ifdef __cplusplus
}
#endif
