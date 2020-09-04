//
// SOFTWARE-ENABLED FLASH (“SEF”) API
// SEFAPI.h
//
// Copyright (C) 2018, 2019, 2020 - KIOXIA Corporation. All rights reserved.
//
// This software is licensed under the 3-Clause BSD License.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted 
// provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following 
// disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
//  disclaimer in the documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products 
// derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

/**
 *  @file SEF Library API
 *
 *  Functions and structures for configuring and using SEF units
 *
 *  @version   1.10c
 *  @date      Aug 2020
 *  @copyright Copyright (C) 2018, 2019, 2020 - KIOXIA Corporation. All rights reserved.
 *
 *  @defgroup ApiManCmd API Management Commands
 *  @defgroup ApiDataCmd Data Access Commands
 *  @defgroup CommonStructs Common Structures
 *  @defgroup CallbackStructs Callback Structures
 *  @defgroup EventsStructs Events
 */

#ifndef SEFAPI_h
#define SEFAPI_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#if !defined(__APPLE__) && !defined(_MSC_VER)
#include <endian.h>
#endif
#if defined(_MSC_VER)
#define PACKED
#define PACKED_ALIGN(A)
#define le64toh(U) (U)
#define htole64(U) (U)
#pragma warning(disable : 4200)  // zero-sized array
#pragma warning(disable : 4201)  // nameless struct/union
#else
#define PACKED __attribute__((packed))
#define PACKED_ALIGN(A) __attribute__((packed,aligned(A)))
#include <sys/uio.h>
#endif

#pragma pack(push,8)

#define SEFAPIVersion 0x010a
#define SEFMaxRootPointer 16
#define SEFMaxFMQueues 8

enum SEFDefectManagementMethod { kPacked, kFragmented, kPerfect } PACKED;
enum SEFAPIIdentifier { kSuperBlock, kInDriveGC, kVirtualSSD } PACKED;
enum SEFErrorRecoveryMode { kAutomatic, kHostControled } PACKED;
enum SEFDeadlineType { kFastest, kTypical, kLong, kHeroic } PACKED;
enum SEFGCMethodType { kStableLatency, kIdleTime, kStop } PACKED;

// definition of bits in supported options field of SEFInfo struct below...
#define kFragmentedSupported            (1 << 0)
#define kPackedSupported                (1 << 1)
#define kPerfectSupported               (1 << 2)
#define kEncryptionSupported            (1 << 3)
#define kHostSerialNumberSupported      (1 << 4)
#define kCopyUserAddressRangeSupported  (1 << 5)
#define kCopyFlashAddressListSupported  (1 << 6)
#define kSuperBlockSupported            (1 << 7)
#define kInDriveGCSupported             (1 << 8)
#define kVirtualSSDSupported            (1 << 9)
#define kAutomaticSupported             (1 << 10)
#define kHostControledSupported         (1 << 11)
#define kFastestSupported               (1 << 12)
#define kTypicalSupported               (1 << 13)
#define kLongSupported                  (1 << 14)
#define kHeroicSupported                (1 << 15)
#define kStableLatencySupported         (1 << 16)
#define kIdleTimeSupported              (1 << 17)
#define kStopSupported                  (1 << 18)

/** @internal */
#define PASTE_TOKENS_(X,Y) X ## Y
/** @internal */
#define PASTE_TOKENS(X,Y) PASTE_TOKENS_(X,Y)

#if defined(_MSC_VER)
#define ALIGN_FOR_LONG
// Used to align structure members based on pointer size
#define ALIGN_FOR_POINTER(currentAlignment)
#else
/**
 *  @internal
 *  Used to align structure to 64bit that's odd-pointer aligned
 */
#define ALIGN_FOR_LONG char PASTE_TOKENS(reserved,__LINE__)[(sizeof(void*)-8)*-1]
/**
 *  @internal
 *  Used to align structure members based on pointer size
 */
#define ALIGN_FOR_POINTER(currentAlignment) \
        char PASTE_TOKENS(reserved,__LINE__)[sizeof(void*) - currentAlignment]
#endif

/**
 *  @ingroup    CommonStructs
 */
struct SEFStatus {
  int64_t error : 32; /**< Status information */
  int64_t info : 32;  /**< Additional context-based descriptive information */
};

typedef struct SEFHandle_ *SEFHandle;
typedef struct SEFVDHandle_ *SEFVDHandle;
typedef struct SEFQoSHandle_ *SEFQoSHandle;

/**
 *  @ingroup    CommonStructs
 */
struct SEFVirtualDeviceID {
  uint16_t id;
};

/**
 *  @ingroup    CommonStructs
 */
struct SEFQoSDomainID {
  uint16_t id;
};

/**
 *  @ingroup    CommonStructs
 */
struct SEFPlacementID {
  uint16_t id;
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Initializes the SEF Library, enumerates the SEF Units present,
 *              and returns the number of units found.
 *
 *  Every successful call to SEFLibraryInit() must be balanced with a call to
 *  SEFLibraryCleanup().
 *
 *  @see        SEFStatus SEFLibraryCleanup()
 *
 *  @return     Status and info summarizing result. The 'info' member contains
 *              number of units.
 */
struct SEFStatus SEFLibraryInit(void);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns a handle to the SEF device at the specified index (zero based)
 *
 *  @param      index        Index of the SEF Unit
 *
 *  @return     Handle to the SEF Unit
 */
SEFHandle SEFGetHandle(int index);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Performs cleanup of the SEF Library and releases resources.
 *
 *  @see        SEFStatus SEFLibraryInit()
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFLibraryCleanup(void);

/**
 *  @ingroup    CommonStructs
 *
 */
struct SEFInfo {
  char vendor[8];               /**< Vendor field */
  char serialNumber[20];        /**< Device serial number */
  char FWVersion[8];            /**< Device firmware version */
  char HWVersion[8];            /**< Device hardware version */
  uint16_t maxQoSDomains;       /**< Hardware version specific, may be less than 65535 defined by architecture */
  uint16_t maxRootPointers;     /**< Firmware version specific, may be less than 16 defined by architecture */
  uint64_t supportedOptions;    /**< Bitmap of supported features */
  uint16_t maxPlacementIDs;     /**< Firmware version specific,   max number of open superblocks per QoS domain */
  uint16_t numFlashMediaQueues; /**< Firmware version specific, max number of scheduling queues per die */
  uint16_t numVirtualDevices;   /**< Number of currently defined virtual devices */
  uint16_t numQoSDomains;       /**< Number of currently defined QoS Domains */
  uint16_t APIVersion;          /**< API Version */
  uint16_t numDies;             /**< Number of dies per channel */
  uint16_t numChannels;         /**< Number of channels per SEF Unit */
  uint16_t numPlanes;           /**< Number of planes per die */
  uint16_t numADUSizes;         /**< Size of ADUsize array that follows at end of structure */
  uint16_t reserved_0;
  uint32_t numBlocks;            /**< Number of blocks per die */
  uint32_t numPages;             /**< Number of pages per block */
  uint32_t pageSize;             /**< Physical page size */
  uint32_t metaSize;             /**< Meta size per ADU */
  uint32_t totalBandWidth;       /**< Total bandwidth corresponding to the underlying NAND component on this device */
  uint32_t readLatency;          /**< Read latency corresponding to the underlying NAND components on this device */
  uint32_t writeLatency;         /**< Write latency corresponding to the underlying NAND components on this device */
  uint32_t eraseLatency;         /**< Erase latency corresponding to the underlying NAND components on this device */
  uint32_t openExpirationPeriod; /**< Granularity in seconds for entire block */
  uint32_t ADUsize[0];           /**< Array of supported ADU sizes (in bytes) */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Gets device information.
 *
 *  Returns ADU size(s), number of channels, number of dies, and other
 *  associated information.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle                Handle to the SEF Unit
 *
 *  @return     Status and info summarizing result.
 */
const struct SEFInfo *SEFGetInformation(SEFHandle sefHandle);

/**
 *  @ingroup    CommonStructs
 */
struct SEFVirtualDeviceList {
  uint16_t numVirtualDevices;                   /**< Number of virtual devices */
  struct SEFVirtualDeviceID virtualDeviceID[0]; /**< An Array of all Virtual device IDs */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns a list of the defined Virtual Devices.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle                Handle to the SEF Unit
 *  @param[out] list                     Buffer for storing list of Virtual Devices
 *  @param      bufferSize               Buffer size
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFListVirtualDevices(SEFHandle sefHandle, struct SEFVirtualDeviceList *list, int bufferSize);

/**
 *  @ingroup    CommonStructs
 */
struct SEFQoSDomainList {
  uint16_t numQoSDomains;               /**< Number of QoS domains */
  struct SEFQoSDomainID QoSDomainID[0]; /**< An Array of all QoS Domain IDs */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns a list of the defined QoS Domains.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle                Handle to the SEF Unit
 *  @param[out] list                     Buffer for storing list of QoS Domains
 *  @param      bufferSize               Buffer size
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFListQoSDomains(SEFHandle sefHandle, struct SEFQoSDomainList *list, int bufferSize);

/**
 *  @ingroup    CommonStructs
 *  @brief      Structure of SEFUserAddress may be redefined by user.
 *
 *  The limitations for redefining the structure are:
 *  1. size must be metaSize from SEFInfo struct, 8 bytes for foreseeable future
 *  2. Value of 0xFFFFFFFFFFFFFFFF is reserved
 *  3. multi-adu writes will auto increment the LBA value
 *
 *  For kSuperblock, the LBA is limited to 40 bits and the meta to 24.
 *  The bits member is in little endian format.
 *
 */
struct SEFUserAddress {
  uint64_t unformatted;
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Return LBA and meta values from a user address
 **/
#define SEFGetUserAddressMeta(U) ((uint32_t) (le64toh((U).unformatted) >> 40))

/**
 *  @ingroup    ApiManCmd
 *  @brief      Return LBA and meta values from a user address
 **/
#define SEFGetUserAddressLba(U) (le64toh((U).unformatted) & ((1ULL << 40)-1))

/**
 *  @ingroup    ApiManCmd
 *  @brief      Return LBA and meta values from a user address
 **/
#define SEFParseUserAddress(U,L,M) do {\
  *(L) = SEFGetUserAddressLba(U); \
  *(M) = SEFGetUserAddressMeta(U); \
} while(0)

/**
 *  @ingroup    CommonStructs
 *  @brief      Number of bits in a user address lba value
 **/
#define SEFUserAddressLbaBits 40

/**
 *  @ingroup    CommonStructs
 *  @brief      Number of bits in a user address meta value
 **/
#define SEFUserAddressMetaBits (64-SEFUserAddressLbaBits)

/**
 *  @ingroup    ApiManCmd
 *  @brief      Creates a user address from lba and meta values
 **/
#define SEFCreateUserAddress(L,M) \
  ((struct SEFUserAddress) {htole64(((uint64_t) (M) << SEFUserAddressLbaBits) \
  | ((L) & ((1UL << SEFUserAddressLbaBits)-1)))})

/**
 *  @ingroup     CommonStructs
 *  @brief       Opaque flash address value parsable by SEFParseFlashAddress()
 */
struct SEFFlashAddress {
  uint64_t bits;
};

#if defined(_MSC_VER)
static inline struct SEFFlashAddress _int2fa(uint64_t v) {return {v};}
#define SEFAutoAllocate _int2fa(0xffffffffffffffffUL)
static inline struct SEFUserAddress _int2ua(uint64_t v) {return {v};}
#define SEFUserAddressIgnore _int2ua(0xffffffffffffffffUL)
#else
/**
 *  @ingroup     CommonStructs
 *  @brief       Flash address value to indicate device should allocate
 *  @see         SEFWriteWithoutPhysicalAddress1
 */
#define SEFAutoAllocate ((struct SEFFlashAddress) {0xffffffffffffffffUL})
/**
 *  @ingroup     CommonStructs
 *  @brief       User address value to indicate it should not be validated by
 *               the SEF device
 *  @see         SEFReadWithPhysicalAddress1
 */
#define SEFUserAddressIgnore ((struct SEFUserAddress) {0xffffffffffffffffUL})
#endif

/**
 *  @ingroup     CommonStructs
 *  @brief       Asynchronous notifications from SEF
 */
enum SEFNotificationType {
  kAddressUpdate,
  kUnflushedData,
  kRequirePatrol,
  kRequireMaintenance,
  kReducedCapacity,
  kUnreadableData,
  kSuperblockStateChanged,
  kOutOfCapacity
} PACKED;

/**
 *  @ingroup    EventsStructs
 *  @brief      This event is issued at the QoS Domain level.
 */
struct SEFQoSNotification {
  enum SEFNotificationType type; /**< See union below... */
  uint8_t reserved_0[5];
  struct SEFQoSDomainID QoSDomainID; /**< QoSDomainID for this notification */
  union {
    struct {
      struct SEFUserAddress changedUserAddress;     /**< User address that moved */
      struct SEFFlashAddress oldFlashAddress;       /**< Old flash address */
      struct SEFFlashAddress newFlashAddress;       /**< New flash address */
    }; /**< kAddressUpdate */
    struct SEFFlashAddress maintenanceFlashAddress; /**< kRequireMaintenance */
    struct SEFFlashAddress patrolFlashAddress;      /**< kRequirePatrol */
    struct {
      char *userData;                               /**< pointer to buffered data */
      struct SEFUserAddress unflushedUserAddress;   /**< affected user address */
    }; /**< kUnflushedData */
    struct SEFFlashAddress unreadableFlashAddress;  /**< kUnreadable */
    struct SEFFlashAddress changedFlashAddress;     /**< kSuperblockStateChanged open=>closed */
  }; /**< Notification data */
};

/**
 *  @ingroup    EventsStructs
 *  @brief      This event indicates to the host that it should respond insome
 *              appropriate manner to the reduced capacity condition.
 *
 * This event is issued at the Virtual Device level.  Due to failure of blocks,
 * actual available capacity may fall below the allocated capacity of the
 * attached QoS Domains. This event indicates to the host that it should
 * respond in some appropriate manner to the reduced capacity condition.
 */
struct SEFVDNotification {
  enum SEFNotificationType type; /**< Is kReducedCapacity or kOutOfCapacity */
  uint8_t reserved_0;
  struct SEFVirtualDeviceID virtualDeviceID; /**< virtual device for this notification */
  uint32_t numADUs;              /**< kReducedCapacity - Amount of space that is no longer available */
};

//
// Management operations associated with device control/configuration
//

/**
 *  @ingroup    CommonStructs
 */
struct SEFDieMap {
  uint8_t startChannel; /**< starting channel number for rectangular region */
  uint8_t startBank;    /**< starting bank number for rectangular region */
  uint8_t numChannels;  /**< width for rectangular region */
  uint8_t numBanks;     /**< height for rectangular region */
};

/**
 *  @ingroup    CommonStructs
 *  @brief      Relative die time weights for basic operations
 */
struct SEFWeights {
  uint16_t readWeight;    /**< Default Weight for a Read operation by Read commands */
  uint16_t eraseWeight;   /**< Default Weight for an Erase operation by SEFAllocateSuperBlock for user Nameless writes */
  uint16_t programWeight; /**< Default Weight for a Program operation by Nameless Write commands */
  uint16_t read4CopyWeight;    /**< Default Weight for a Read operation by Nameless Copy commands */
  uint16_t erase4CopyWeight;   /**< Default Weight for an Erase operation by SEFAllocateSuperBlock for Nameless Copy */
  uint16_t program4CopyWeight; /**< Default Weight for a Program operation by Nameless Copy commands */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Creates a Virtual Device and allocates physical resources.
 *
 *  @see        SEFStatus SEFGetInformation()
 *
 *  @param      sefHandle            Handle to the SEF Unit
 *  @param      virtualDeviceID      Virtual Device ID
 *  @param      dieMap               Dies requested for virtual device
 *  @param      defectStrategy       Defect management strategy for the Virtual Device
 *  @param      numFMQueues          Number of Flash Media Queues per die in the Virtual Device
 *  @param      weights              Weight values for each Flash Media Queue
 *
 *  @return     Status and info summarizing result. Returns 0 on success and negative value on error.
 */
struct SEFStatus SEFCreateVirtualDevice(SEFHandle sefHandle, struct SEFVirtualDeviceID virtualDeviceID,
                                        struct SEFDieMap dieMap,
                                        enum SEFDefectManagementMethod defectStrategy, uint8_t numFMQueues,
                                        const struct SEFWeights weights[]);

/**
 *  @ingroup    CommonStructs
 */
struct SEFVirtualDeviceInfo {
  uint64_t flashCapacity;                   /**< Flash capacity in ADUs */
  uint64_t flashAvailable;                  /**< Available flash capacity in ADUs */
  uint32_t superBlockCapacity;              /**< Total SuperBlock capacity in ADUs */
  uint32_t eraseCount;                      /**< Number of superblocks erased.  Used to populate
                                                 eraseOrder in SEFSuperBlockRecord */
  struct SEFDieMap dieMap;                  /**< Dies allocated to this virtual device */
  struct SEFWeights weights[SEFMaxFMQueues];/**< Weights for each FMQ */
  uint8_t numFMQueues;                      /**< Number of flash media queues per die */
  enum SEFDefectManagementMethod defectStrategy; /**<  Defect management strategy for the Virtual Device */
  uint8_t averagePEcount;                   /**< Average program/erase count */
  uint8_t maxPEcount;                       /**< Max program/erase count */
  uint16_t numUnallocatedSuperBlocks;       /**< Number of unallocated super blocks */
  uint16_t numSuperBlocks;                  /**< Number of allocated super blocks */
  struct SEFQoSDomainList QoSDomains;       /**< List of domains */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns Virtual Device information.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle            Handle to the SEF Unit
 *  @param      virtualDeviceID      Virtual Device ID
 *  @param[out] info                 Buffer for storing VD information
 *  @param      bufferSize           Buffer size
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFGetVirtualDeviceInformation(SEFHandle sefHandle, struct SEFVirtualDeviceID virtualDeviceID,
                                                struct SEFVirtualDeviceInfo *info, int bufferSize);

/**
 *  @ingroup    CommonStructs
 */
struct SEFFMQAssignments {
  uint8_t readFMQ;         /**< Default FMQ for user read commands */
  uint8_t programFMQ;      /**< Default FMQ for user nameless write commands */
  uint8_t read4CopyFMQ;    /**< Default FMQ for read by nameless copy commands */
  uint8_t program4CopyFMQ; /**< Default FMQ for write by nameless copy commands */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Attempts to create a QoS Domain in the specified Virtual Device.
 *
 *  Returns an error when the target virtual device doesn’t have enough flash
 *  memory space. When the flashQuota is less than the flashCapacity, it will
 *  be set to the flashCapacity.
 *
 *  @see        SEFGetInformation()
 *
 *  @param      vdHandle             Handle to the Virtual Device
 *  @param      QoSDomainID          QoS Domain ID. Unique across all QoS Domains
 *  @param      flashCapacity        Number of required/reserved ADUs
 *  @param      flashQuota           Number of ADUs that can be allocated
 *  @param      ADUsize              Size of ADU in this QoS domain in bytes.
 *                                   Must be one of the values in ADUSize[]
 *                                   in SEFInfo returned by SEFGetInformation().
 *                                   SEF Unit should support 4kiB.
 *  @param      api                  Specifies the API Identifier for this QoS domain
 *  @param      recovery             Specifies the recovery mode for this QoS domain
 *  @param      encryption           0 for disabled, non-zero for enabled
 *  @param      numRootPointers      Specifies the number of root pointers
 *                                   corresponding to this QoS domain
 *  @param      numPlacementIDs      The maximum number of Placement IDs that
 *                                   can be placed on the QoS domain. (The
 *                                   number of Placement IDs would affect
 *                                   memory usage)
 *  @param      FMQDefaults          The default die FMQ assignments for I/O
 *                                   commands
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFCreateQoSDomain(SEFVDHandle vdHandle, struct SEFQoSDomainID QoSDomainID, uint64_t flashCapacity,
                                    uint64_t flashQuota, uint32_t ADUsize, enum SEFAPIIdentifier api,
                                    enum SEFErrorRecoveryMode recovery, int encryption,
                                    uint16_t numRootPointers, uint16_t numPlacementIDs,
                                    struct SEFFMQAssignments FMQDefaults);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Resets the capacity of a QoS Domain
 *
 *  Sets a new capacity and quota for the QoS domain. When the flashQuota is
 *  less than the flashCapacity, it is set to the flashCapacity. Returns an
 *  error when the total capacity of assigned super blocks exceeds the new
 *  capacity.
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param      flashCapacity    Number of required/reserved ADUs
 *  @param      flashQutoa       Number of ADUs that can be allocated
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFSetQoSDomainCapacity(SEFQoSHandle qosHandle, uint64_t flashCapacity,
                                         uint64_t flashQutoa);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Sets the physical address of the QoSDomain root ADU pointer.
 *
 *  A root pointer may be set to any value. Root pointer values are read back
 *  using SEFGetQoSDomainInformation().  When a root pointer is set to a flash
 *  address that is valid for the QoS domain it's stored in, the ADU it points to
 *  can be read by SEFReadWithPhysicalAddress1() using a flash address of just
 *  the root pointer index as the ADU.
 *
 *  @see        SEFStatus SEFReadWithPhysicalAddress1()
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param      index            The index of the root pointer
 *  @param      value            Value of the pointer
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFSetRootPointer(SEFQoSHandle qosHandle, int index, struct SEFFlashAddress value);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Sets target QoS Domain's read deadline policy.
 *
 *  @see        SEFStatus SEFVirtualDeviceInfo
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param      deadline         Deadline type for this QoS domain
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFSetReadDeadline(SEFQoSHandle qosHandle, enum SEFDeadlineType deadline);

/**
 *  @ingroup    CommonStructs
 */
struct SEFSuperBlockRecord {
  struct SEFFlashAddress flashAddress; /**< Flash address where this superblock resides */
  uint32_t eraseOrder;                /**< Indication of when a superblock was erased.
                                           Can be used to determine the order blocks were
                                           allocated or to version a superblock. Values
                                           only increase over time and are unique at the
                                           virtual device level */
  uint32_t writableADUs;              /**< If superblock is closed, writableADUs and
                                           writtenADUs are equal; if they are not equal,
                                           the superblock must still be open  */
  uint32_t writtenADUs;               /**< This field increments as ADUs in the superblock are written */
  struct SEFPlacementID placementID;  /**< When auto-allocated, indicates the placement id
                                           supplied to SEFWriteWithoutPhysicalAddress1().
                                           Otherwise it will be 0xffff */
  uint8_t PEIndex;                    /**< This is the block's erase count normalized
                                           to be between 0 and 255 */
};

/**
 *  @ingroup    CommonStructs
 */
struct SEFQoSDomainInfo {
  struct SEFVirtualDeviceID virtualDeviceID; /**< Virtual device ID */
  uint16_t numPlacementIDs;               /**< Specifies the number of Placement IDs corresponding to this QoS domain */
  uint16_t numRootPointers;               /**< Specifies the number of root pointers corresponding to this QoS domain */
  uint8_t encryption;                     /**< 0 for disabled, non-zero for enabled */
  enum SEFAPIIdentifier api;              /**< Specifies the API Identifier for this QoS domain */
  uint64_t capacity;                      /**< Reserved capacity of the QoS domain in ADUs */
  uint64_t quota;                         /**< Number of ADUs that can be allocated by the QoS domain */
  enum SEFErrorRecoveryMode recoveryMode; /**< Specifies the recovery mode for this QoS domain */
  enum SEFDeadlineType deadline;          /**< Deadline type for the QoS domain */
  struct SEFFMQAssignments FMQDefaults;   /**< The default die FMQ assignments for I/O commands */
  uint16_t reserved_0;
  struct SEFFlashAddress rootPointers[SEFMaxRootPointer]; /**< List of root pointers */
  uint32_t ADUsize;                       /**< Size of ADU in bytes */
  uint32_t numSuperBlocks;                /**< Number of superblocks in use by the QoS Domain */
  struct SEFSuperBlockRecord superBlockRecords[0]; /**< List of superblock records */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns QoS Domain information, including the list of super
 *              blocks assigned to the QoS Domain.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      QoSDomainID      QoS Domain ID
 *  @param[out] info             Buffer for storing QoS Domain information
 *  @param      bufferSize       Buffer size
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFGetQoSDomainInformation(SEFHandle sefHandle, struct SEFQoSDomainID QoSDomainID,
                                            struct SEFQoSDomainInfo *info, int bufferSize);

/**
 *  @ingroup    CommonStructs
 */
struct SEFWearInfo {
  uint32_t numSuperBlocks; /**< Number of superblocks */
  uint32_t reserved_0;
  struct SEFSuperBlockRecord superBlockRecords[0]; /**< List of superblock records */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns list of SuperBlocks to process for wear-leveling.
 *
 *  Used in support of the implementation of a host-specified wear leveling
 *  policy. SEF has a built in wear-leveling mechanism.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param[out] info             Buffer for storing information of blocks to process
 *  @param      bufferSize       Buffer size
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFGetReuseList(SEFQoSHandle qosHandle, struct SEFWearInfo *info, int bufferSize);

/**
 *  @ingroup    CommonStructs
 */
struct SEFRefreshInfo {
  uint32_t numSuperBlocks; /**< Number of superblocks */
  uint32_t reserved_0;
  struct SEFSuperBlockRecord superBlockRecords[0]; /**< List of superblock records */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns a list of blocks that have encountered ECC errors.
 *
 *  These blocks subsequently need to be re-written, or else data loss may
 *  occur. This call should be part of a periodic background check to guard
 *  against data loss.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param[out] info             Buffer for storing information of  blocks to process
 *  @param      bufferSize       Buffer size
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFGetRefreshList(SEFQoSHandle qosHandle, struct SEFRefreshInfo *info, int bufferSize);

/**
 *  @ingroup    CommonStructs
 *  @brief      Super blocks returned by SEFGetCheckList()
 */
struct SEFCheckInfo {
  uint32_t numSuperBlocks; /**< Number of superblocks */
  uint32_t reserved_0;
  struct SEFSuperBlockRecord superBlockRecords[0]; /**< List of superblock records */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns a list of blocks that have encountered conditions that
 *              need to be checked.
 *
 *  In the event that this command indicates that blocks need to be
 *  checked, a subsequent patrol command (SEFCheckPage) should be issued in
 *  response.  Detailed error statistics will be returned as part of the patrol,
 *  and appropriate corrective actions can be based on the returned information.
 *
 *  @see        SEFStatus SEFCheckPage()
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param[out] info             Buffer for storing information of blocks to process
 *  @param      bufferSize       Buffer size
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFGetCheckList(SEFQoSHandle qosHandle, struct SEFCheckInfo *info, int bufferSize);

/**
 *  @ingroup    CommonStructs
 */
struct SEFUserAddressRecovery {
  uint64_t serial; /**< Monotonically increasing generational counter that indicates the order in which blocks were
                      written. For example, it can be used for replay for data recovery */
  struct SEFUserAddress userAddress; /**< Contains LBA information */
};

/**
 *  @ingroup    CommonStructs
 */
struct SEFUserAddressRecord {
  uint32_t numADUs; /**< Number of ADUs */
  uint32_t reserved_0;
  struct SEFUserAddressRecovery userAddressesRecovery[0]; /**< User address recovery scheme */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns the user address list in terms of its underlying superblocks.
 *
 *  Used as part of an FTL reconstruction activity. This can happen in the
 *  event of, for example, ungraceful shutdown.  This mechanism can also be
 *  used to build custom diagnostic tools. This command is not needed during
 *  normal operation.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param      flashAddress     Physical address of the superblock
 *  @param[out] list             Buffer for storing list of user addresses
 *  @param      bufferSize       Buffer size
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFGetUserAddressList(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress,
                                       struct SEFUserAddressRecord *list, int bufferSize);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Returns information corresponding to the superblock.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param      flashAddress     Physical address of the superblock
 *  @param[out] info             Buffer for storing superblock information
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFGetSuperBlockInfo(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress,
                                      struct SEFSuperBlockRecord *info);

/**
 *  @ingroup    ApiManCmd
 *  @brief      This is a read patrol operation which can be used in conjunction
 *              with SEFGetCheckList.
 *
 *  Returns detailed information concerning checked pages to
 *  allow host software to take appropriate corrective actions.
 *
 *  @see        SEFStatus SEFGetCheckList()
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param      flashAddress     Physical address to be checked
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFCheckPage(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Deletes the target virtual device.
 *
 *  The Virtual Device must be in the closed state before issuing this command.
 *  Moreover, this command will fail if the Virtual Device contains any QoS
 *  Domains.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      virtualDeviceID  Virtual Device ID
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFDeleteVirtualDevice(SEFHandle sefHandle, struct SEFVirtualDeviceID virtualDeviceID);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Deletes the target QoS domain.
 *
 *  The QoS domain must be in the closed state before issuing this command.
 *  After closing the target QoS domain, its assigned superblocks are returned
 *  to the virtual device's free pool.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      QoSDomainID      QoS Domain ID
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFDeleteQoSDomain(SEFHandle sefHandle, struct SEFQoSDomainID QoSDomainID);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Resets the encryption key for a QoS Domain.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      QoSDomainID      QoS Domain ID
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFResetEncryptionKey(SEFHandle sefHandle, struct SEFQoSDomainID QoSDomainID);

//
// Normal user operations associated with I/O and the data path
//

/**
 *  @ingroup    ApiManCmd
 *  @brief      Opens the target virtual device.
 *
 *  Since Virtual Devices are persistent, this provides the mechanism for
 *  opening a preexisting Virtual Device to resume I/O after reboot. This
 *  function needs to be called in order to receive notifications about the
 *  virtual device, such as in the event that a reduced capacity notification
 *  is issued.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      virtualDeviceID  Virtual Device ID
 *  @param      notifyFunc       Callback to be executed upon event generation
 *  @param      context          A void*  pointer passed to the async event notification
 *                               function (used to pass user context information)
 *  @param      vdHandle         Handle to the Virtual Drive
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFOpenVirtualDevice(SEFHandle sefHandle, struct SEFVirtualDeviceID virtualDeviceID,
                                      void (*notifyFunc)(void *, struct SEFVDNotification), void *context,
                                      SEFVDHandle *vdHandle);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Closes an open Virtual Device and shuts down associated event notification.
 *
 *  @see        SEFStatus
 *
 *  @param      vdHandle         Handle to the Virtual Device
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFCloseVirtualDevice(SEFVDHandle vdHandle);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Opens a previously created QoS Domain.
 *
 *  Since QoS Domains are persistent, this provides the mechanism for opening
 *  a preexisting QoS Domain to resume I/O after reboot.  This function also
 *  provides a channel to receive notifications regarding this QoS domain.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      QoSDomainID      QoS Domain ID
 *  @param      notifyFunc       Callback to be executed during event generation
 *  @param      context          A void*  pointer passed to the async event notification
 *                               function (used to pass user context information)
 *  @param      encryptionKey    In a multitenant environment, different tenants will
 *                               write to separate QoS domains.  Provides for individualized
 *                               encryption keys on a per-domain basis
 *  @param[out] qosHandle        Handle to the QoS Domain
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFOpenQoSDomain(SEFHandle sefHandle, struct SEFQoSDomainID QoSDomainID,
                                  void (*notifyFunc)(void *, struct SEFQoSNotification), void *context,
                                  const void *encryptionKey, SEFQoSHandle *qosHandle);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Closes an open QoS Domain.
 *
 *  This in turn will close any open superblocks associated with this domain.
 *  All outstanding kSuperblockChangeState events will be delivered before this
 *  function returns.  A QoS Domain must be in the closed state to be deleted.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFCloseQoSDomain(SEFQoSHandle qosHandle);

/**
 *  @ingroup    ApiManCmd
 *  @brief      This function is used to extract info needed by FTL from
 *              an opaque flash address.
 *
 *  @see        SEFStatus
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      flashAddress     The opaque address to be parsed
 *  @param[out] QoSDomainID      A pointer to where to return the QoS Domain ID. A null pointer
 *                               indicates that the Qos Domain ID is not to be returned
 *  @param[out] blockNumber      A pointer to where to return the block number. A null pointer
 *                               indicates that the block number is not to be returned
 *  @param[out] ADUOffset        A pointer to where to return the ADU Offset. A null pointer
 *                               indicates that the ADU Offset is not to be returned
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFParseFlashAddress(SEFHandle sefHandle, struct SEFFlashAddress flashAddress,
                                      struct SEFQoSDomainID *QoSDomainID, uint16_t *blockNumber, uint32_t *ADUOffset);

/**
 *  @ingroup    ApiManCmd
 *  @brief      This function is used to create an opaque flash address.
 *
 * A generated flash address may be rejected by the device if it specifies an
 * illegal ADUOffset, a block number not owned by the QoSDomainID, or a
 * QoSDomainID that has not been opened by the caller.
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      QoSDomainID       The desired QoS Domain ID.
 *  @param      blockNumber       The desired block number.
 *  @param      ADUOffset         The desired ADU Offset.
 *
 *  @return     The generated flash address.
 */
struct SEFFlashAddress SEFCreateFlashAddress(SEFHandle sefHandle,
                                             struct SEFQoSDomainID QoSDomainID,
                                             uint16_t blockNumber,
                                             uint32_t ADUOffset);

/*
 * following routines have sync and async interfaces. QoS Domain must be in the open state
 */

/**
 *  @ingroup    CommonStructs
 *  @brief Supplied to override default write FMQ and weights
 *
 *  May be used when calling SEFWriteWithoutPhysicalAddress1() or
 *  SEFWriteWithoutPhysicalAddress1Async(). Any of these fields can be set
 *  to -1 to use the default
 */
struct SEFWriteOverrides {
  uint16_t eraseWeight;   /**< Weight to use for erase instead of virtual device default */
  uint16_t programWeight; /**< Weight to use for program instead of virtual device default */
  uint8_t programFMQ;     /**< Flash Media Queue to use for erase & write instead of QoS Domain default */
};

/**
 *  @ingroup    ApiDataCmd
 *  @brief      Writes data to the specified user address to an underlying
 *              physical flash page that is assigned for the QoS Domain.
 *
 *  If auto-allocate was enabled on the superblock, when the assigned superblock
 *  is filled and closed, SEF assigns a new super-block for following writes. If
 *  auto-allocate is not enabled, host software will know about the superblock
 *  size as part of the allocation, and can use this information to construct
 *  appropriately-sized write commands. Manually allocated superblocks for
 *  writes MUST be of type kForWrite. This call will not return until the data
 *  has been persisted, and will automatically pad the user data with dummy data
 *  if required to complete flash memory programming.
 *
 *  The userAddress supplied here will be checked when reading the data back
 *  with SEFReadWithPhysicalAddress1().  In kSuperblock mode, the LBA portion of
 *  the user address is incremented for each ADU when writing multiple ADUs.
 *  The user address value 0xFFFFFFFFFFFFFFFF is reserved and is invalid.
 *
 *  @note The synchronous and asynchronous versions differ in how data is
 *  committed to flash. As described above, the synchronous version flushes data
 *  to flash returning permanent flash addresses. In contrast, the asynchronous
 *  version lazily flushes data to flash. The flash addresses returned are
 *  tentative instead. Once the SEF device eventually flushes a tentative
 *  address to flash it may be discovered to be bad. When this happens, a
 *  kAddressUpdate QoS notification is sent indicating the data has moved to a
 *  new permanent flash address. There is no notification for addresses that
 *  have successfully flushed and are now permanent. It can be inferred instead
 *  by the kSuperblockStateChanged QoS notification for the owning superblock.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle                  Handle to the QoS Domain
 *  @param      flashAddress               Physical address of the superblock. 0xFFFFFFFFFFFFFFFF if auto allocate.
 *  @param      placementID                Only valid if the flashAddress is auto allocated. A value from 0 to
 *                                         numPlacementIds–1 indicating what logical data group to place this data in.
 *  @param      userAddress                FTL can store meta-data related to this operation by this field. For
 *                                         example, storing LBA address to bind to this write operation such as data
 *                                         tags.
 *  @param      numADU                     Total amount of write data size calculated in ADU.  Maximum allowed is
 *                                         64k ADUs.
 *  @param      iov                        A pointer to the scatter gather list
 *  @param      iovcnt                     The number of elements in the scatter gather list
 *  @param[out] permanentAddresses         Must allocate space for returned permanent addresses equal to 8*length
 *                                         (e.g. 8*number of ADUs)
 *  @param[out] distanceToEndOfSuperBlock  Indicates remaining size in ADU after this write operation.  May be NULL.
 *                                         This is not a guarantee as the block may be forced closed if too many
 *                                         superblocks are open.
 *  @param      overrides                  Overrides to scheduler parameters; pointer can be null for none required.
 *
 *  @return     Status and info summarizing result.  When .error is non-zero, .info is the number of ADUs written.
 */
struct SEFStatus SEFWriteWithoutPhysicalAddress1(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress,
                                                 struct SEFPlacementID placementID, struct SEFUserAddress userAddress,
                                                 uint32_t numADU, const struct iovec *iov, uint16_t iovcnt,
                                                 struct SEFFlashAddress *permanentAddresses,
                                                 uint32_t *distanceToEndOfSuperBlock,
                                                 const struct SEFWriteOverrides *overrides);

/**
 *  @ingroup    CommonStructs
 *  @brief Supplied to override default read FMQ and weight
 *
 *  May be used when calling SEFReadWithPhysicalAddress1() or
 *  SEFReadWithPhysicalAddress1Async(). Any of these fields can be set to -1 to
 *  use the default
 */
struct SEFReadOverrides {
  uint16_t readWeight; /**< Weight to use for read instead of virtual device default */
  uint8_t readFMQ;     /**< Flash Media Queue to use for read instead of QoS Domain default */
};

/**
 *  @ingroup    ApiDataCmd
 *  @brief      Reads data from a specified physical address.
 *
 *  While writes are expressed in terms of logical addresses, reads are
 *  expressed in terms of physical addresses.  Read commands may interrupt other
 *  types of commands. When there is an in-flight flash memory command to the
 *  same flash die other than a read command, the in-flight command will be
 *  suspended in order to maintain deterministic read latency.  If the target
 *  physical address is currently in the process of being programmed, data will
 *  instead be returned from the write buffer.
 *
 *  The userAddress must either match what was stored when the data was written
 *  or be ~0 to disable checking.  In kSuperblock mode, the LBA portion of the
 *  user address is incremented for each ADU in a multi-adu write.
 *
 *  @note When reading data that was just written, a read error will be returned
 *  when the data's original flash address has been updated but the notification
 *  has yet to be processed by the client.  In this case, the caller must retry
 *  the read after the changed flash address notification has been processed.
 *
 *  @see        SEFStatus SEFSetRootPointer()
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param      flashAddress     Physical address for the read command; When the
 *                               QoS domain ID and block number are 0, the ADU
 *                               offset is the root pointer index for the flash
 *                               address to read.
 *  @param      numADU           Length of data to read (in ADUs). Maximum allowed
 *                               is superblockCapacity.
 *  @param      iov              A pointer to the scatter gather list
 *  @param      iovcnt           Number of elements in the scatter gather list
 *  @param      iovOffset        Starting byte offset into iov array
 *  @param      userAddress      Stored data by the FTL.  It will be validated
 *                               with what was stored when the data was written
 *                               except when SEFUserAddressIgnore is supplied
 *  @param      overrides        Overrides to scheduler parameters; pointer can
 *                               be null for none required.
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFReadWithPhysicalAddress1(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress,
                                             uint32_t numADU, const struct iovec *iov, uint16_t iovcnt,
                                             uint32_t iovOffset, struct SEFUserAddress userAddress,
                                             const struct SEFReadOverrides *overrides);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Releases the specific Super Block to the free pool owned by the
 *              Virtual Device to which the specified QoS Domain belongs.
 *
 *  The target superblock must have been assigned by a previous call to
 *  SEFAllocateSuperBlock() or as part of SEFWriteWithoutPhysicalAddress1().
 *  The superblock must be closed, otherwise the call will fail.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle        Handle to the QoS Domain of the Super Block
 *  @param      flashAddress     Physical address of the superblock to release
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFReleaseSuperBlock(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress);

/**
 *  @ingroup    CommonStructs
 *  @brief Supplied to override default superblock allocation FMQ and weight
 *
 *  May be used when calling SEFAllocateSuperBlock() or
 *  SEFAllocateSuperBlockAsync(). Any of these fields can be set to -1 to use
 *  the default
 */
struct SEFAllocateOverrides {
  uint16_t eraseWeight; /**< Weight to use for erase instead of virtual device default */
  uint8_t eraseFMQ;     /**< Flash Media Queue to use for erase instead of QoS Domain default */
};

enum SEFSuperBlockType {
  kForWrite,         //**< Superblock is for writes */
  kForCopy,          //**< Superblock is for a nameless copy */
  kForDeviceMetadata //**< Superblock is for device metadata (for device-internal use) */
} PACKED;

/**
 *  @ingroup    ApiManCmd
 *  @brief      Allocates a superblock that will be assigned to the specified
 *              QoS Domain and returns the physical address of this superblock.
 *
 *  Any number of superblocks can be kept open for write for each QoS domain.
 *  These superblocks in turn can be used as part of the parameter set for
 *  SEFWriteWithoutPhysicalAddress(). When allocating a superblock, SEF
 *  intelligently selects a location in a manner  designed to optimize the
 *  lifetime of flash memory and will return the physical address that was
 *  selected.  Note that each open superblock will allocate a write buffer and
 *  therefore consume memory, so there is a tradeoff in the number of open
 *  superblocks and the amount of memory consumed.
 *
 *  Required that the total ADUs in the domain be less than its flash quota.
 *  This can be known by summing the writableADUs of each superblock in the
 *  domain.
 *
 *  @see        SEFStatus SEFGetQoSDomainInformation()
 *
 *  @param      qosHandle        Handle to the QoS Domain
 *  @param[out] flashAddress     The flash address of the allocated block
 *  @param      retention        Retention period in hours
 *  @param      type             kForWrite, kForCopy or kForDeviceMetadata
 *  @param      overrides        Overrides to scheduler parameters; pointer
 *                               can be null for none required.
 *
 *  @return     Status and info summarizing result.
 *              .info contains number of ADUs in allocated superblock
 */
struct SEFStatus SEFAllocateSuperBlock(SEFQoSHandle qosHandle, struct SEFFlashAddress *flashAddress, uint32_t retention,
                                       enum SEFSuperBlockType type, const struct SEFAllocateOverrides *overrides);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Flushes the target superblock.
 *
 *  This command causes all written data for the superblock that is still in the write buffer
 *  and not persisted to flash memory to be persisted to flash memory. The device will automatically append
 *  data if necessary to finish programming of all pending user data writes. This command will
 *  not return until any address change notifications for the superblock being flushed have been
 *  processed, ensuring that all previously tentative addresses are now permanent.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle        Handle to the QoS Domain of the Super Block
 *  @param      flashAddress     Physical address of the Super Block to be
 *                               flushed.
 *  @param[out] distanceToEndOfSuperBlock  Indicates remaining size in ADU after
 *              this flush operation.  May be NULL.
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFFlushSuperBlock(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress,
                                    uint32_t *distanceToEndOfSuperBlock);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Closes the target superblock.
 *
 *  If there is remaining unwritten space in the superblock, that space will be
 *  padded with dummy data.  This can be used by the FTL as a means of
 *  closing a superblock without invoking a Write command.
 *
 *  This command will not return until all address change and superblock state
 *  change notifications for the superblock being closed have been processed,
 *  ensuring that all previously tentative addresses are now permanent.
 *
 *  @see        SEFStatus
 *
 *  @param      qosHandle        Handle to the QoS Domain of the Super Block
 *  @param      flashAddress     Physical address of the Super Block to move
 *                               to Closed state by filling data
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SEFCloseSuperBlock(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress);

enum SEFCopySourceType { kBitmap, kList } PACKED;

/**
 *  @ingroup     CommonStructs
 *  @brief       Source addresses for SEFNamelessCopy().
 *
 *  The Source addresses format controls if the validBitmap or list of
 *  flash addresses is used.
 *
 *  @see SEFNamelessCopy() SEFUserAddressFilter
 */
struct SEFCopySource {
  enum SEFCopySourceType format;              /**< Specifies the format to use */
  uint8_t reserved_0[3];
  uint32_t arraySize; /**< Number of items in bitmap array or Flash Address List (QWORD count) */
  union {
    struct {
      struct SEFFlashAddress srcFlashAddress; /**< flash address of source block.  ADU & ~0x3f indicates the ADU of
                                                   bit 0 of validBitmap and ADU & 0x3f is the starting bit in validBiMap */
      uint64_t *validBitmap;                  /**< pointer to COPY of valid bitmap array (little endian), memory allocated
                                                   by SEFPrepareBufferForNamelessCopy() */
    };
    struct SEFFlashAddress *flashAddressList; /**< pointer to flash address list, memory allocated by
                                                   SEFPrepareBufferForNamelessCopy() */
  };
};

/**
 *  @ingroup     CommonStructs
 *  @brief       Optional filtering on user address data during copy
 */
struct SEFUserAddressFilter {
  struct SEFUserAddress userAddressStart; /**< Starting user address of filter */
  uint64_t userAddressRangeLength;        /**< Length of filter range (0 indicates no filtering) */
  uint32_t userAddressRangeType;          /**< Zero to copy data in range; non-zero to copy outside of range */
};

/**
 *  @ingroup    CommonStructs
 *  @brief      Address change records
 *
 *  This structure is used internally to implement SEFProcessChangeAddressRequest().
 *  It may change in the future so it should be treated as opaque.
 */
struct SEFAddressChangeRequest {
  uint32_t numProcessedADUs;
  uint32_t nextADUOffset;
  uint32_t numReadErrorADUs;
  uint16_t numDefectivePlanes;
  uint16_t reserved;
  struct SEFFlashAddress startingDstFlashAddress;
  struct SEFUserAddress userAddress[0];
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      This function allocates a data buffer SEFNamelessCopy().
 *
 *  It initializes filter parameters and returns pointers into members of
 *  copySource and addressChangeInfo.
 *
 *  @param      copySource               Description of copy source; format and arraysize
 *                                       MUST be initialized. The validBitmap pointer or
 *                                       flashAddressList pointer will be set by this function.
 *  @param      filter                   Pointer to filter parameters, may be null for no
 *                                       filtering. This function will set the filter fields
 *                                       in the buffer.
 *  @param      numAddressChangeRecords  Size of addressChangeRequest userAddress array
 *  @param[out] addressChangeInfo        A pointer to pointer to the address change info
 *                                       within the buffer (set by this function)
 *
 *  @return     Pointer to allocated buffer or NULL if error
 */
void *SEFPrepareBufferForNamelessCopy(struct SEFCopySource *copySource, const struct SEFUserAddressFilter *filter,
                                      uint32_t numAddressChangeRecords,
                                      struct SEFAddressChangeRequest **addressChangeInfo);

/**
 *  @ingroup    ApiManCmd
 *  @brief      Frees the buffer allocated with SEFPrepareBufferForNamelessCopy().
 *
 *  @param      copyContext                A pointer to the memory to free
 */
void SEFFreeBufferForNamelessCopy(void *copyContext);

/**
 *  @ingroup    CommonStructs
 *  @brief      Flash Meida Queue overrides for SEFNamelessCopy()
 *
 *  When any of these fields are set to ~0, the default weight is used as
 *  defined by SEFCreateVirtualDevice() and default FMQ as defined by
 *  SEFCreateQoSDomain().
 */
struct SEFCopyOverrides {
  uint16_t readWeight;    /**< Weight to use for read instead of virtual device default */
  uint16_t eraseWeight;   /**< Weight to use for erase instead of virtual device default */
  uint16_t programWeight; /**< Weight to use for program instead of virtual device default */
  uint8_t readFMQ;        /**< Flash Media Queue to use for read instead of QoS Domain default */
  uint8_t programFMQ;     /**< Flash Media Queue to use for erase & write instead of QoS Domain default */
};

#define kCopyConsumedSource             (1 << 0)  /**< Flag set in status.info field for SEFNamelessCopy() \
                                                       when all source ADUs were copied */
#define kCopyClosedDestination          (1 << 1)  /**< Flag set in status.info field for SEFNamelessCopy() \
                                                       when the destination superblock was filled/closed */
#define kCopyFilteredUserAddresses      (1 << 2)  /**< Flag set in status.info field for SEFNamelessCopy() \
                                                       when data outside the user address filter was detected */
#define kCopyReadErrorOnSource          (1 << 3)  /**< Flag set in status.info field for SEFNamelessCopy() \
                                                       when some of the source ADUs were not copied because of a read error */
#define kCopyDesinationDefectivePlanes  (1 << 4)  /**< Flag set in status.info field for SEFNamelessCopy() \
                                                       when destination superblock has defective planes */

/**
 *  @ingroup    ApiDataCmd
 *  @brief      Performs Nameless Copy with map or list; optional user address filtering.
 *
 *  Copies ADUs as described by copySource to the copyDestination.  If the
 *  destination superblock was allocated by SEFAllocateSuperBlock() the type
 *  must be kForCopy.
 *
 *  @note Padding is added when the copy is not a multiple of the minimum
 *  writeable unit.
 *
 *  @see        SEFStatus SEFProcessAddressChangeRequests() SEFPrepareBufferForNamelessCopy()
 *
 *  @param      srcQosHandle                 Handle to the source QoS Domain
 *  @param      copySource                   Physical addresses to copy
 *  @param      dstQosHandle                 Handle to the destination QoS Domain
 *  @param      copyDestination              Physical address of destination superblock
 *  @param      filter                       Pointer to user address filter parameters,
 *                                           null indicates no filtering
 *  @param      overrides                    Pointer to overrides to scheduler parameters;
 *                                           pointer can be null for none required.
 *  @param      numAddressChangeRecords      Maximum number of ADUs to copy (size of
 *                                           addressChangeRequest userAddress array)
 *  @param[out] addressChangeInfo            Information to record changed addresses
 *  @param      copyContext                  Pointer to working buffer
 *                                           returned by SEFPrepareBufferForNamelessCopy()
 *  @return     Status and info summarizing result, .info contains:
 *                - Destination super block has defective planes (1bit)
 *                - Read error was detected  on source (1bit)
 *                - Data that is out of User Address range is detected (1bit)
 *                - Destination superblock was filled/closed (1bit)
 *                - Consumed entire source bitmap or list (1bit)
 */
struct SEFStatus SEFNamelessCopy(SEFQoSHandle srcQosHandle, struct SEFCopySource copySource, SEFQoSHandle dstQosHandle,
                                 struct SEFFlashAddress copyDestination, const struct SEFUserAddressFilter *filter,
                                 const struct SEFCopyOverrides *overrides, uint32_t numAddressChangeRecords,
                                 struct SEFAddressChangeRequest *addressChangeInfo, void *copyContext);

/**
 *  @ingroup    ApiDataCmd
 *  @brief      Performs post processing of address change records for Nameless Copy.
 *
 *  @see        SEFStatus SEFNamelessCopy()
 *
 *  @param      srcQosHandle                 Handle to the source QoS Domain
 *  @param      copySource                   Physical addresses to copy
 *  @param      dstQosHandle                 Handle to the destination QoS Domain
 *  @param      copyInfo                     Information returned from namelessCopy in status.info field.
 *              copyInfo contains:
 *                - Destination super block has defective planes (1bit)
 *                - Read error was detected  on source (1bit)
 *                - Data that is out of User Address range is detected (1bit)
 *                - Destination superblock was filled/closed (1bit)
 *                - Consumed entire source bitmap or list (1bit)
 *  @param      addressChangeInfo            Information to record changed addresses
 *
 *  @return     Status and info summarizing result.
 *
 */
struct SEFStatus SEFProcessAddressChangeRequests(SEFQoSHandle srcQosHandle, struct SEFCopySource copySource,
                                                 SEFQoSHandle dstQosHandle, uint32_t copyInfo,
                                                 const struct SEFAddressChangeRequest *addressChangeInfo);

/**
 *  @ingroup    CallbackStructs
 */
struct SEFWriteWithoutPhysicalAddressIOCB {
  struct SEFStatus status;              /**< Library sets error field to a non-zero value to
                                             indicate any error when a command completes */
  int16_t opcode;                       /**< Should never be accessed - for internal use by library */
  int16_t done;                         /**< Flag for polled I/O - library sets this field to a
                                             non-zero value once the command completes */
  ALIGN_FOR_POINTER(4);
  void *param1;                         /**< Ignored by the library; the caller can store context
                                             information that may be accessed from the completion
                                             function */
  void (*complete_func)(
      struct SEFWriteWithoutPhysicalAddressIOCB *); /**< If non-zero, treated as the address of a function to
                                                         be called when a command completes */
  struct SEFFlashAddress *tentativeAddresses; /**< List of tentative addresses return  */
  const struct SEFWriteOverrides *overrides;  /**< Override parameters for scheduling purposes, may be NULL */
  struct SEFFlashAddress flashAddress;  /**< Address of the superblock for this write; -1 for
                                              auto-allocate, or can use value from previous
                                              superblock allocation call */
  struct SEFUserAddress userAddress;    /**< Contains LBA information */
  const struct iovec *iov;              /**< A pointer to the scatter gather list */
  uint16_t iovcnt;                      /**< number of elements in the scatter gather list */
  struct SEFPlacementID placementID;    /**< Only valid if the flashAddress is auto allocated. A
                                             value from 0 to numPlacementIds – 1 indicating what
                                             logical data group to place this data in */
  uint32_t numADU;                      /**< Length in ADUs, maximum is 64k ADUs */
  uint32_t distanceToEndOfSuperBlock;   /**< Return value in units of ADUs */
};

/**
 *  @ingroup    ApiDataCmd
 *  @brief      This function is the asynchronous version of SEFWriteWithoutPhysicalAddress1().
 *
 *  @note Any kAddressUpdate and kSuperBlockStateChange QoS notifications for
 *  the returned tentative addresses will occur after the iocb completion
 *  routine has returned.  When no completion routine is set, the caller must
 *  handle the race condition of acting on done being set and the notifications
 *  being sent.
 *
 *  @see        SEFWriteWithoutPhysicalAddress1()
 *
 *  @param          qosHandle    Handle to the QoS Domain
 *  @param[in,out]  iocb         For asynchronous response from SEF Library.
 *                               Unused fields should be set to 0.
 */
void SEFWriteWithoutPhysicalAddress1Async(SEFQoSHandle qosHandle, struct SEFWriteWithoutPhysicalAddressIOCB *iocb);

/**
 *  @ingroup    CallbackStructs
 */
struct SEFReadWithPhysicalAddressIOCB {
  struct SEFStatus status;              /**< Library sets error field to a non-zero value to indicate
                                             any error when a command completes */
  int16_t opcode;                       /**< Should never be accessed - for internal use by library */
  int16_t done;                         /**< Flag for polled I/O - library sets this field to a
                                             non-zero value once the command completes */
  ALIGN_FOR_POINTER(4);
  void *param1;                         /**< Ignored by the library; the caller can store context
                                             information that may be accessed from the completion function */
  void (*complete_func)
    (struct SEFReadWithPhysicalAddressIOCB *); /**< If non-zero, treated as the address of a function
                                                    to be called when a command completes */
  const struct SEFReadOverrides *overrides;    /**< Override parameters for scheduling purposes, may be NULL */
  struct SEFFlashAddress flashAddress;  /**< Physical address for the read command; When the
                                             QoS domain ID and block number are 0, the ADU
                                             offset is the root pointer index for the flash
                                             address to read.*/
  struct SEFUserAddress userAddress;    /**< Contains LBA information */
  const struct iovec *iov;              /**< A pointer to the scatter gather list */
  uint32_t iovOffset;                   /**< Starting byte offset into iov array */
  uint32_t numADU;                      /**< Number of ADUs to be read, maximum is superblockCapacity */
  uint16_t iovcnt;                      /**< Number of elements in the scatter gather list */
};

/**
 *  @ingroup    ApiDataCmd
 *  @brief      This function is the asynchronous version of SEFReadWithPhysicalAddress1().
 *
 *  @see        SEFReadWithPhysicalAddress1()
 *
 *  @param          qosHandle    Handle to the QoS Domain
 *  @param[in,out]  iocb         For asynchronous response from SEF Library
 *                               Unused fields should be set to 0.
 *
 */
void SEFReadWithPhysicalAddress1Async(SEFQoSHandle qosHandle, struct SEFReadWithPhysicalAddressIOCB *iocb);

/**
 *  @ingroup    CallbackStructs
 */
struct SEFReleaseSuperBlockIOCB {
  struct SEFStatus status;              /**< Library sets error field to a non-zero value to
                                             indicate any error when a command completes */
  int16_t opcode;                       /**< Should never be accessed - for internal use by library */
  int16_t done;                         /**< Flag for polled I/O - library sets this field to a
                                             non-zero value once the command completes */
  ALIGN_FOR_POINTER(4);
  void *param1;                         /**< Ignored by the library; the caller can store context
                                             information that may be accessed from the completion function */
  void (*complete_func)
    (struct SEFReleaseSuperBlockIOCB *); /**< If non-zero, treated as the address of a
                                              function to be called when a command completes */
  ALIGN_FOR_LONG;
  struct SEFFlashAddress flashAddress;  /**< Address of superblock */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      This function is the asynchronous version of SEFReleaseSuperBlock().
 *
 *  @see        SEFReleaseSuperBlock()
 *
 *  @param          qosHandle    Handle to the QoS Domain
 *  @param[in,out]  iocb         For asynchronous response from SEF Library
 *                               Unused fields should be set to 0.
 */
void SEFReleaseSuperBlockAsync(SEFQoSHandle qosHandle, struct SEFReleaseSuperBlockIOCB *iocb);

/**
 *  @ingroup    CallbackStructs
 *  @brief IOCB for SEFAllocateSuperBlockAsync()
 */
struct SEFAllocateSuperBlockIOCB {
  struct SEFStatus status;                /**< Library sets error field to a non-zero value to
                                               indicate any error when a command completes */
  int16_t opcode;                         /**< Should never be accessed - for internal use by library */
  int16_t done;                           /**< Flag for polled I/O - library sets this field to a
                                               non-zero value once the command completes */
  ALIGN_FOR_POINTER(4);
  void *param1;                           /**< Ignored by the library; the caller can store context
                                               information that may be accessed from the completion function */
  void (*complete_func)
    (struct SEFAllocateSuperBlockIOCB *); /**< If non-zero, treated as the address of a function to
                                               be called when a command completes */
  const struct SEFAllocateOverrides *overrides; /**< Override parameters for scheduling purposes, may be NULL */
  struct SEFFlashAddress flashAddress;    /**< Address of superblock */
  uint32_t retention;                     /**< Desired retention period in hours */
  enum SEFSuperBlockType type;            /**< kForWrite, kForCopy or kForDeviceMetadata */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      This function is the asynchronous version of SEFAllocateSuperBlock().
 *
 *  @see        SEFAllocateSuperBlock()
 *
 *  @param          qosHandle    Handle to the QoS Domain
 *  @param[in,out]  iocb         For asynchronous response from SEF Library
 *                               Unused fields should be set to 0.
 */
void SEFAllocateSuperBlockAsync(SEFQoSHandle qosHandle, struct SEFAllocateSuperBlockIOCB *iocb);

/**
 *  @ingroup    CallbackStructs
 *  @brief      IOCB for SEFCloseSuperBlockAsync()
 */
struct SEFCloseSuperBlockIOCB {
  struct SEFStatus status;              /**< Library sets error field to a non-zero value to indicate
                                             any error when a command completes */
  int16_t opcode;                       /**< Should never be accessed - for internal use by library */
  int16_t done;                         /**< Flag for polled I/O - library sets this field to a
                                             non-zero value once the command completes */
  ALIGN_FOR_POINTER(4);
  void *param1;                         /**< Ignored by the library; the caller can store context information
                                             that may be accessed from the completion function */
  void (*complete_func)
    (struct SEFCloseSuperBlockIOCB *);  /**< If non-zero, treated as the address of a function to be
                                             called when a command completes */
  struct SEFFlashAddress flashAddress;  /**< Address of the superblock */
};

/**
 *  @ingroup    ApiManCmd
 *  @brief      This function is the asynchronous version of SEFCloseSuperBlock().
 *
 *  kSuperblockStateChanged will have been sent before the completion routine
 *  is called and the iocb is marked as done.
 *
 *  @see        SEFCloseSuperBlock()
 *
 *  @param          qosHandle    Handle to the QoS Domain
 *  @param[in,out]  iocb         For asynchronous response from SEF Library
 *
 */
void SEFCloseSuperBlockAsync(SEFQoSHandle qosHandle, struct SEFCloseSuperBlockIOCB *iocb);

/**
 *  @ingroup    CallbackStructs
 */
struct SEFNamelessCopyIOCB {
  struct SEFStatus status;          /**< Library sets error field to a non-zero value to indicate
                                         any error when a command completes.  See SEFNamelessCopy()
                                         for details of the info field. */
  int16_t opcode;                   /**< Should never be accessed - for internal use by library */
  int16_t done;                     /**< Flag for polled I/O - library sets this field to a
                                         non-zero value once the command completes */
  ALIGN_FOR_POINTER(4);
  void *param1;                     /**< Ignored by the library; the caller can store context
                                         information that may be accessed from the completion function */
  void (*complete_func)
    (struct SEFNamelessCopyIOCB *); /**< If non-zero, treated as the address of a function to be
                                         called when a command completes */
  SEFQoSHandle dstQosHandle;        /**< Handle to the destination QoS Domain */
  struct SEFFlashAddress copyDestination; /**< Physical address of destination superblock */
  struct SEFAddressChangeRequest *
    addressChangeInfo;              /**< Information to record changed addresses */
  uint32_t numAddressChangeRecords; /**< Maximum number of ADUs to copy (size of
                                         addressChangeRequest userAddress array) */
  uint32_t reserved_0;
  struct SEFCopySource copySource;  /**< Physical addresses to copy */
  const struct SEFUserAddressFilter *filter;/**< Pointer to user address filter parameters, null for no filtering */
  const struct SEFCopyOverrides *overrides; /**< Override parameters for scheduling purposes, may be NULL */
  void *copyContext;                /**< Working buffer returned by SEFPrepareBufferForNamelessCopy() */
};

/**
 *  @ingroup    ApiDataCmd
 *  @brief      This function is the asynchronous version of SEFNamelessCopy().
 *
 *  @see        SEFNamelessCopy()
 *
 *  @param          qosHandle    Handle to the source QoS Domain
 *  @param[in,out]  iocb         For asynchronous response from SEF Library
 *                               Unused fields should be set to 0.
 */
void SEFNamelessCopyAsync(SEFQoSHandle qosHandle, struct SEFNamelessCopyIOCB *iocb);

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* SEFAPI_h */
