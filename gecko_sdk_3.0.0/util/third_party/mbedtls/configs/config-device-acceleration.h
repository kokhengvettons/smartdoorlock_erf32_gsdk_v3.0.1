/***************************************************************************//**
 * @brief Configuration for enabling hardware acceleration for mbedtls features.
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

/**
 * @defgroup sl_config_device_acceleration Silicon Labs Hardware Acceleration Configuration
 * @addtogroup sl_config_device_acceleration
 *
 * @brief
 *  mbed TLS configuration for Silicon Labs device specific hardware acceleration
 *
 * @details
 *  mbed TLS configuration is composed of settings in this Silicon Labs device specific hardware acceleration
 *  file located in mbedtls/configs that will enable hardware acceleration of all features where this is
 *  supported. This file should be included from an application specific configuration file that
 *  configures what mbedtls features should be included.
 *
 * @{
 */

#ifndef MBEDTLS_CONFIG_DEVICE_ACCELERATION_H
#define MBEDTLS_CONFIG_DEVICE_ACCELERATION_H

#if !defined(NO_CRYPTO_ACCELERATION)
#include "em_device.h"
/**
 * @name SECTION: Silicon Labs Acceleration settings
 *
 * This section sets Silicon Labs Acceleration settings.
 * @{

 */

/**
 * \def MBEDTLS_AES_ALT
 *
 * Enable hardware acceleration for the AES block cipher
 *
 * Module:  sl_crypto/src/crypto_aes.c for devices with CRYPTO,
 *          sl_crypto/src/se_aes.c for devices with SE,
 *          sl_crypto/src/cryptoacc_aes.c for devices with CRYPTOACC,
 *          sl_crypto/src/aes_aes.c for devices with AES
 *
 * See MBEDTLS_AES_C for more information.
 */
#define MBEDTLS_AES_ALT

/**
 * \def MBEDTLS_ECP_INTERNAL_ALT
 * \def ECP_SHORTWEIERSTRASS
 * \def MBEDTLS_ECP_ADD_MIXED_ALT
 * \def MBEDTLS_ECP_DOUBLE_JAC_ALT
 * \def MBEDTLS_ECP_NORMALIZE_JAC_MANY_ALT
 * \def MBEDTLS_ECP_NORMALIZE_JAC_ALT
 *
 * Enable hardware acceleration for the elliptic curve over GF(p) library.
 *
 * Module:  sl_crypto/src/crypto_ecp.c
 *
 * Caller:  library/ecp.c
 *
 * Requires: MBEDTLS_BIGNUM_C, MBEDTLS_ECP_C and at least one
 * MBEDTLS_ECP_DP_XXX_ENABLED and (CRYPTO_COUNT > 0)
 */
#if defined(CRYPTO_COUNT) && (CRYPTO_COUNT > 0)
#define MBEDTLS_ECP_INTERNAL_ALT
#define ECP_SHORTWEIERSTRASS
#define MBEDTLS_ECP_ADD_MIXED_ALT
#define MBEDTLS_ECP_DOUBLE_JAC_ALT
#define MBEDTLS_ECP_NORMALIZE_JAC_MANY_ALT
#define MBEDTLS_ECP_NORMALIZE_JAC_ALT
#define MBEDTLS_ECP_RANDOMIZE_JAC_ALT
#endif

/**
 * \def MBEDTLS_SHA1_ALT
 *
 * Enable hardware acceleration for the SHA1 cryptographic hash algorithm.
 *
 * Module:  sl_crypto/src/crypto_sha.c for devices with CRYPTO,
 *          sl_crypto/src/se_sha.c for devices with SE,
 *          sl_crypto/src/cryptoacc_sha.c for devices with CRYPTOACC
 *
 * Caller:  library/mbedtls_md.c
 *          library/ssl_cli.c
 *          library/ssl_srv.c
 *          library/ssl_tls.c
 *          library/x509write_crt.c
 *
 * Requires: MBEDTLS_SHA1_C and (CRYPTO_COUNT > 0 or CRYPTOACC_PRESENT or SEMAILBOX_PRESENT)
 *
 * See MBEDTLS_SHA1_C for more information.
 */
#if defined(CRYPTO_COUNT) && (CRYPTO_COUNT > 0)
#define MBEDTLS_SHA1_ALT
#endif

/**
 * \def MBEDTLS_SHA256_ALT
 *
 * Enable hardware acceleration for the SHA-224 and SHA-256 cryptographic
 * hash algorithms.
 *
 * Module:  sl_crypto/src/crypto_sha.c for devices with CRYPTO,
 *          sl_crypto/src/se_sha.c for devices with SE,
 *          sl_crypto/src/cryptoacc_sha.c for devices with CRYPTOACC
 *
 * Caller:  library/entropy.c
 *          library/mbedtls_md.c
 *          library/ssl_cli.c
 *          library/ssl_srv.c
 *          library/ssl_tls.c
 *
 * Requires: MBEDTLS_SHA256_C and (CRYPTO_COUNT > 0 or CRYPTOACC_PRESENT or SEMAILBOX_PRESENT)
 *
 * See MBEDTLS_SHA256_C for more information.
 */
#if defined(CRYPTO_COUNT) && (CRYPTO_COUNT > 0)
#define MBEDTLS_SHA256_ALT
#endif

/* EFR32xG22 Hardware Acceleration */

#if defined(CRYPTOACC_PRESENT) || defined(DOXY_DOC_ONLY)
#define MBEDTLS_AES_ALT
#define AES_192_SUPPORTED
/**
 * \def MBEDTLS_CCM_ALT
 *
 * Enable hardware acceleration CCM.
 *
 * Module:  sl_crypto/src/se_ccm.c for devices with SE,
 *          sl_crypto/src/cryptoacc_ccm.c for devices with CRYPTOACC
 *
 * Requires: MBEDTLS_AES_C and MBEDTLS_CCM_C (CRYPTOACC_PRESENT or SEMAILBOX_PRESENT)
 *
 * See MBEDTLS_CCM_C for more information.
 */
#define MBEDTLS_CCM_ALT
/**
 * \def MBEDTLS_CMAC_ALT
 *
 * Enable hardware acceleration CMAC.
 *
 * Module:  sl_crypto/src/se_cmac.c for devices with SE,
 *          sl_crypto/src/cryptoacc_cmac.c for devices with CRYPTOACC
 *
 * Requires: MBEDTLS_AES_C and MBEDTLS_CMAC_C (CRYPTOACC_PRESENT or SEMAILBOX_PRESENT)
 *
 * See MBEDTLS_CMAC_C for more information.
 */
#define MBEDTLS_CMAC_ALT
/**
 * \def MBEDTLS_GCM_ALT
 *
 * Enable hardware acceleration GCM.
 *
 * Module:  sl_crypto/src/se_gcm.c for devices with SE,
 *          sl_crypto/src/cryptoacc_gcm.c for devices with CRYPTOACC
 *
 * Requires: MBEDTLS_GCM_C (CRYPTOACC_PRESENT or SEMAILBOX_PRESENT)
 *
 * See MBEDTLS_GCM_C for more information.
 */
#define MBEDTLS_GCM_ALT
#define MBEDTLS_SHA1_ALT
#define MBEDTLS_SHA256_ALT

/**
 * \def MBEDTLS_ECDH_COMPUTE_SHARED_ALT
 * \def MBEDTLS_ECDH_GEN_PUBLIC_ALT
 * \def MBEDTLS_ECDSA_GENKEY_ALT
 * \def MBEDTLS_ECDSA_SIGN_ALT
 * \def MBEDTLS_ECDSA_VERIFY_ALT
 *
 * Enable hardware acceleration ECP.
 *
 * Module:  sl_crypto/src/se_ecp.c for devices with SE,
 *          sl_crypto/src/cryptoacc_ecp.c for devices with CRYPTOACC
 *
 * Requires: MBEDTLS_ECP_C (CRYPTOACC_PRESENT or SEMAILBOX_PRESENT)
 *
 * See MBEDTLS_ECP_C for more information.
 */
#if !(   defined(MBEDTLS_ECP_DP_SECP384R1_ENABLED)        \
      || defined(MBEDTLS_ECP_DP_SECP521R1_ENABLED)        \
      || defined(MBEDTLS_ECP_DP_SECP192K1_ENABLED)        \
      || defined(MBEDTLS_ECP_DP_SECP224K1_ENABLED)        \
      || defined(MBEDTLS_ECP_DP_SECP256K1_ENABLED)        \
      || defined(MBEDTLS_ECP_DP_BP256R1_ENABLED)          \
      || defined(MBEDTLS_ECP_DP_BP384R1_ENABLED)          \
      || defined(MBEDTLS_ECP_DP_BP512R1_ENABLED)          \
      || defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)       \
      || defined(MBEDTLS_ECP_DP_CURVE448_ENABLED) )
  #define MBEDTLS_ECDH_COMPUTE_SHARED_ALT
  #define MBEDTLS_ECDH_GEN_PUBLIC_ALT
  #define MBEDTLS_ECDSA_GENKEY_ALT
  #define MBEDTLS_ECDSA_SIGN_ALT
  #define MBEDTLS_ECDSA_VERIFY_ALT
#endif // #if !(   defined(MBEDTLS_ECP_DP_XXX_ENABLED) ...

#endif /* CRYPTOACC */


#if defined(SEMAILBOX_PRESENT) || defined(DOXY_DOC_ONLY)
#include "em_se.h"

#if defined(SE_COMMAND_OPTION_HASH_SHA1)
#define MBEDTLS_SHA1_ALT
#define MBEDTLS_SHA1_PROCESS_ALT
#endif
#if defined(SE_COMMAND_OPTION_HASH_SHA256) || defined(SE_COMMAND_OPTION_HASH_SHA224)
#define MBEDTLS_SHA256_ALT
#define MBEDTLS_SHA256_PROCESS_ALT
#endif
#if defined(SE_COMMAND_OPTION_HASH_SHA512) || defined(SE_COMMAND_OPTION_HASH_SHA384)
#define MBEDTLS_SHA512_ALT
#define MBEDTLS_SHA512_PROCESS_ALT
#endif

#if  !defined(MBEDTLS_ECP_DP_SECP224R1_ENABLED)                         \
  && !defined(MBEDTLS_ECP_DP_SECP192K1_ENABLED)                         \
  && !defined(MBEDTLS_ECP_DP_SECP224K1_ENABLED)                         \
  && !defined(MBEDTLS_ECP_DP_SECP256K1_ENABLED)                         \
  && !defined(MBEDTLS_ECP_DP_BP256R1_ENABLED)                           \
  && !defined(MBEDTLS_ECP_DP_BP384R1_ENABLED)                           \
  && !defined(MBEDTLS_ECP_DP_BP512R1_ENABLED)                           \
  && !defined(MBEDTLS_ECP_DP_CURVE448_ENABLED)

  #if ! ( (_SILICON_LABS_SECURITY_FEATURE == _SILICON_LABS_SECURITY_FEATURE_SE) \
          && defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED) )
    #if defined(SE_COMMAND_CREATE_KEY)
      #define MBEDTLS_ECDH_GEN_PUBLIC_ALT
      #define MBEDTLS_ECDSA_GENKEY_ALT
    #endif

    #if defined(SE_COMMAND_DH)
      #define MBEDTLS_ECDH_COMPUTE_SHARED_ALT
    #endif

  #endif

  #if defined(SE_COMMAND_SIGNATURE_SIGN) \
    && !defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
    #define MBEDTLS_ECDSA_SIGN_ALT
  #endif

  #if defined(SE_COMMAND_SIGNATURE_VERIFY) \
    && !defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
    #define MBEDTLS_ECDSA_VERIFY_ALT
  #endif

#else // #if !defined(MBEDTLS_ECP_DP_XXXX_ENABLED) && ...

#error UNSUPPORTED curve included

#endif // #if !defined(MBEDTLS_ECP_DP_XXXX_ENABLED) && ...

#if defined(SE_COMMAND_JPAKE_GEN_SESSIONKEY) || defined(DOXY_DOC_ONLY)
/**
 * \def MBEDTLS_ECJPAKE_ALT
 *
 * Enable hardware acceleration JPAKE.
 *
 * Module:  sl_crypto/src/se_jpake.c
 *
 * Requires: MBEDTLS_ECJPAKE_C (SEMAILBOX_PRESENT)
 *
 * See MBEDTLS_ECJPAKE_C for more information.
 */
#define MBEDTLS_ECJPAKE_ALT
#endif

#if defined(SE_COMMAND_AES_CCM_ENCRYPT) && defined(SE_COMMAND_AES_CCM_DECRYPT)
#define MBEDTLS_CCM_ALT
#endif
#if defined(SE_COMMAND_AES_CMAC)
#define MBEDTLS_CMAC_ALT
#endif
#endif /* SEMAILBOX_PRESENT */

/**
 * \def MBEDTLS_TRNG_PRESENT
 *
 * Decode if device supports the True Random Number Generator (TRNG)
 * incorporated from Series 1 Configuration 2 devices (EFR32MG12, etc.)
 * from Silicon Labs.
 *
 * TRNG is not supported by software for EFR32XG13 (SDID_89) and
 * EFR32XG14 (SDID_95).
 *
 * Requires TRNG_PRESENT &&
 *  !(_SILICON_LABS_GECKO_INTERNAL_SDID_95) &&
 *  !(_SILICON_LABS_GECKO_INTERNAL_SDID_89)
 */
#if defined(TRNG_PRESENT) && \
  !defined(_SILICON_LABS_GECKO_INTERNAL_SDID_95) && \
  !defined(_SILICON_LABS_GECKO_INTERNAL_SDID_89)
#undef MBEDTLS_TRNG_PRESENT
#define MBEDTLS_TRNG_PRESENT
#endif

/**
 * \def MBEDTLS_ENTROPY_ADC_PRESENT
 *
 * Decode if device supports retrieving entropy data from the ADC
 * incorporated on devices from Silicon Labs.
 *
 * Requires ADC_PRESENT && _ADC_SINGLECTRLX_VREFSEL_VENTROPY &&
 *          _SILICON_LABS_32B_SERIES_1
 */
#if defined(ADC_PRESENT) && \
    defined(_ADC_SINGLECTRLX_VREFSEL_VENTROPY) && \
    defined(_SILICON_LABS_32B_SERIES_1)
#define MBEDTLS_ENTROPY_ADC_PRESENT
#endif

/**
 * \def MBEDTLS_ENTROPY_RAIL_C
 *
 * Decode if device supports retrieving entropy data from RAIL.
 *
 * Requires _EFR_DEVICE && _SILICON_LABS_32B_SERIES_1)
 */
#if defined(_EFR_DEVICE) && defined(_SILICON_LABS_32B_SERIES_1)
#define MBEDTLS_ENTROPY_RAIL_PRESENT
#endif

/* Default ECC configuration for Silicon Labs devices: */

/* Save RAM by adjusting to our exact needs */
#ifndef MBEDTLS_ECP_MAX_BITS
#define MBEDTLS_ECP_MAX_BITS   256
#endif
#ifndef MBEDTLS_MPI_MAX_SIZE
#define MBEDTLS_MPI_MAX_SIZE    32 // 384 bits is 48 bytes
#endif

/*
   Set MBEDTLS_ECP_WINDOW_SIZE to configure
   ECC point multiplication window size, see ecp.h:
   2 = Save RAM at the expense of speed
   3 = Improve speed at the expense of RAM
   4 = Optimize speed at the expense of RAM
*/
#define MBEDTLS_ECP_WINDOW_SIZE        2
#define MBEDTLS_ECP_FIXED_POINT_OPTIM  0

/* Significant speed benefit at the expense of some ROM */
#define MBEDTLS_ECP_NIST_OPTIM

#endif /* !NO_CRYPTO_ACCELERATION */

/** @} (end section sl_config_device_acceleration) */
/** @} (end addtogroup sl_config_device_acceleration) */

#endif /* MBEDTLS_CONFIG_DEVICE_ACCELERATION_H */
