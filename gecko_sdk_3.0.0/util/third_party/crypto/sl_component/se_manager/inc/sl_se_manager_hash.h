/***************************************************************************//**
 * @file
 * @brief Silicon Labs Secure Element Manager API.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: APACHE-2.0
 *
 * This software is subject to an open source license and is distributed by
 * Silicon Laboratories Inc. pursuant to the terms of the Apache License,
 * Version 2.0 available at https://www.apache.org/licenses/LICENSE-2.0.
 * Such terms and conditions may be further supplemented by the Silicon Labs
 * Master Software License Agreement (MSLA) available at www.silabs.com and its
 * sections applicable to open source software.
 *
 ******************************************************************************/
#ifndef SL_SE_MANAGER_HASH_H
#define SL_SE_MANAGER_HASH_H

#include "em_device.h"

#if defined(SEMAILBOX_PRESENT) || defined(DOXYGEN)

/// @addtogroup sl_se_manager
/// @{

/***************************************************************************//**
 * @addtogroup sl_se_manager_hash Hashing
 *
 * @brief
 *   Provides cryptographic hash functions (SHA-1, SHA-224, SHA-256, SHA-384,
 *   SHA-512).
 *
 * @details
 *   Provides API for one-way hashing functions.
 *
 * @{
 ******************************************************************************/

#include "sl_se_manager_key_handling.h"
#include "sl_se_manager_types.h"
#include "em_se.h"
#include "sl_status.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Prototypes

/***************************************************************************//**
 * @brief
 *   Produce a message digest (a hash block) using the input data.
 *
 * @details
 *   This function generates a message digest adhering to the given inputs.
 *   For instance, if the algorithm is chosen to be SHA-256, it will generate
 *   a 32 bytes message digest computed based on the input message.
 *   This function supports SHA-1, SHA-256 and SHA-512 algorithms.
 *
 * @param[in] cmd_ctx
 *   Pointer to an SE command context object.
 *
 * @param[in] hash_type
 *   Which hashing algorithm to use.
 *
 * @param[in] message
 *   Pointer to the message buffer to compute the hash/digest from.
 *
 * @param[in] message_size
 *   Number of bytes in message.
 *
 * @param[out] digest
 *   Pointer to block of memory to store the final digest.
 *
 * @param[in]  digest_len
 *   The length of the message digest (hash), must be at least the size of the
 *   corresponding hash type.
 *
 * @return Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash(sl_se_command_context_t *cmd_ctx,
                       sl_se_hash_type_t hash_type,
                       const uint8_t *message,
                       unsigned int message_size,
                       uint8_t* digest,
                       size_t digest_len);

/***************************************************************************//**
 * @brief
 *   Prepare a SHA1 hash streaming command context object.
 *
 * @details
 *   Prepare a SHA1 hash streaming command context object to be used in
 *   subsequent calls to hash streaming functions sl_se_hash_update() and
 *   sl_se_hash_finish().
 *
 * @param[in] hash_ctx
 *   Pointer to a generic hash streaming context object.
 *
 * @param[in] cmd_ctx
 *   Pointer to an SE command context object.
 *
 * @param[in] sha1_ctx
 *   Pointer to a SHA1 streaming context object.
 *
 * @return
 *   Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash_sha1_starts(sl_se_hash_streaming_context_t *hash_ctx,
                                   sl_se_command_context_t *cmd_ctx,
                                   sl_se_sha1_streaming_context_t *sha1_ctx);

/***************************************************************************//**
 * @brief
 *   Prepare a SHA224 hash streaming command context object.
 *
 * @details
 *   Prepare a SHA224 hash streaming command context object to be used in
 *   subsequent calls to hash streaming functions sl_se_hash_update() and
 *   sl_se_hash_finish().
 *
 * @param[in] hash_ctx
 *   Pointer to a generic hash streaming context object.
 *
 * @param[in] cmd_ctx
 *   Pointer to an SE command context object.
 *
 * @param[in] sha224_ctx
 *   Pointer to a SHA224 streaming context object.
 *
 * @return
 *   Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash_sha224_starts(sl_se_hash_streaming_context_t *hash_ctx,
                                     sl_se_command_context_t *cmd_ctx,
                                     sl_se_sha224_streaming_context_t *sha224_ctx);

/***************************************************************************//**
 * @brief
 *   Prepare a SHA256 hash streaming command context object.
 *
 * @details
 *   Prepare a SHA256 hash streaming command context object to be used in
 *   subsequent calls to hash streaming functions sl_se_hash_update() and
 *   sl_se_hash_finish().
 *
 * @param[in] hash_ctx
 *   Pointer to a generic hash streaming context object.
 *
 * @param[in] cmd_ctx
 *   Pointer to an SE command context object.
 *
 * @param[in] sha256_ctx
 *   Pointer to a SHA256 streaming context object.
 *
 * @return
 *   Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash_sha256_starts(sl_se_hash_streaming_context_t *hash_ctx,
                                     sl_se_command_context_t *cmd_ctx,
                                     sl_se_sha256_streaming_context_t *sha256_ctx);

#if (_SILICON_LABS_SECURITY_FEATURE == _SILICON_LABS_SECURITY_FEATURE_VAULT) || defined(DOXYGEN)
/***************************************************************************//**
 * @brief
 *   Prepare a SHA384 streaming command context object.
 *
 * @details
 *   Prepare a SHA384 hash streaming command context object to be used in
 *   subsequent calls to hash streaming functions sl_se_hash_update() and
 *   sl_se_hash_finish().
 *
 * @param[in] hash_ctx
 *   Pointer to a generic hash streaming context object.
 *
 * @param[in] cmd_ctx
 *   Pointer to an SE command context object.
 *
 * @param[in] sha384_ctx
 *   Pointer to a SHA384 streaming context object.
 *
 * @return
 *   Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash_sha384_starts(sl_se_hash_streaming_context_t *hash_ctx,
                                     sl_se_command_context_t *cmd_ctx,
                                     sl_se_sha384_streaming_context_t *sha384_ctx);

/***************************************************************************//**
 * @brief
 *   Prepare a SHA512 streaming command context object.
 *
 * @details
 *   Prepare a SHA512 hash streaming command context object to be used in
 *   subsequent calls to hash streaming functions sl_se_hash_update() and
 *   sl_se_hash_finish().
 *
 * @param[in] hash_ctx
 *   Pointer to a generic hash streaming context object.
 *
 * @param[in] cmd_ctx
 *   Pointer to an SE command context object.
 *
 * @param[in] sha512_ctx
 *   Pointer to a SHA512 streaming context object.
 *
 * @return
 *   Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash_sha512_starts(sl_se_hash_streaming_context_t *hash_ctx,
                                     sl_se_command_context_t *cmd_ctx,
                                     sl_se_sha512_streaming_context_t *sha512_ctx);
#endif // (_SILICON_LABS_SECURITY_FEATURE == _SILICON_LABS_SECURITY_FEATURE_VAULT)

/***************************************************************************//**
 * @brief
 *   Prepare a hash streaming command context object.
 *
 * @details
 *   Prepare a hash (message digest) streaming command context object to be
 *   used in subsequent calls to hash streaming functions sl_se_hash_update()
 *   and sl_se_hash_finish().
 *
 * @param[in] hash_ctx
 *   Pointer to a generic hash streaming context object.
 *
 * @param[in] cmd_ctx
 *   Pointer to an SE command context object.
 *
 * @param[in] hash_type
 *   Type of hash algoritm
 *
 * @param[in] hash_type_ctx
 *   Pointer to a hash streaming context object specific to the hash type
 *   specified by @p hash_type.
 *
 * @return
 *   Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash_starts(sl_se_hash_streaming_context_t *hash_ctx,
                              sl_se_command_context_t *cmd_ctx,
                              sl_se_hash_type_t hash_type,
                              void *hash_type_ctx);

/***************************************************************************//**
 * @brief
 *   Feeds an input buffer into an ongoing hash computation.
 *
 *   This function is called between @ref sl_se_hash_starts() and
 *   @ref sl_se_hash_finish().
 *   This function can be called repeatedly.
 *
 * @param[in] hash_ctx
 *   Pointer to a generic hash streaming context object.
 *
 * @param[in] input
 *   Buffer holding the input data, must be at least @p ilen bytes wide.
 *
 * @param[in] input_len
 *   The length of the input data in bytes.
 *
 * @return
 *   Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash_update(sl_se_hash_streaming_context_t *hash_ctx,
                              const uint8_t *input,
                              size_t input_len);

/***************************************************************************//**
 * @brief
 *   Finish a hash streaming operation and return the resulting hash digest.
 *
 *   This function is called after sl_se_hash_update().
 *
 * @param[in] hash_ctx
 *   Pointer to a generic hash streaming context object.
 *
 * @param[out] digest_out
 *   Buffer for holding the message digest (hash), must be at least the size
 *   of the corresponding message digest type.
 *
 * @param[in]  digest_len
 *   The length of the message digest (hash), must be at least the size of the
 *   corresponding hash type.
 *
 * @return
 *   Status code, @ref sl_status.h.
 ******************************************************************************/
sl_status_t sl_se_hash_finish(sl_se_hash_streaming_context_t *hash_ctx,
                              uint8_t *digest_out,
                              size_t digest_len);

#ifdef __cplusplus
}
#endif

/// @} (end addtogroup sl_se_manager_hash)
/// @} (end addtogroup sl_se_manager)

#endif // defined(SEMAILBOX_PRESENT)

#endif // SL_SE_MANAGER_HASH_H
