

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ServoControl.idl
using RTI Code Generator (rtiddsgen) version 3.1.1.2.
The rtiddsgen tool is part of the RTI Connext DDS distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the Code Generator User's Manual.
*/

#ifndef ServoControlPlugin_1848045218_h
#define ServoControlPlugin_1848045218_h

#include "ServoControl.h"

struct RTICdrStream;

#ifndef pres_typePlugin_h
#include "pres/pres_typePlugin.h"
#endif

#if (defined(RTI_WIN32) || defined (RTI_WINCE) || defined(RTI_INTIME)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

extern "C"{

    #define ServoControlPlugin_get_sample PRESTypePluginDefaultEndpointData_getSample 

    #define ServoControlPlugin_get_buffer PRESTypePluginDefaultEndpointData_getBuffer 
    #define ServoControlPlugin_return_buffer PRESTypePluginDefaultEndpointData_returnBuffer

    #define ServoControlPlugin_create_sample PRESTypePluginDefaultEndpointData_createSample 
    #define ServoControlPlugin_destroy_sample PRESTypePluginDefaultEndpointData_deleteSample 

    /* --------------------------------------------------------------------------------------
    Support functions:
    * -------------------------------------------------------------------------------------- */

    NDDSUSERDllExport extern ServoControl*
    ServoControlPluginSupport_create_data_w_params(
        const struct DDS_TypeAllocationParams_t * alloc_params);

    NDDSUSERDllExport extern ServoControl*
    ServoControlPluginSupport_create_data_ex(RTIBool allocate_pointers);

    NDDSUSERDllExport extern ServoControl*
    ServoControlPluginSupport_create_data(void);

    NDDSUSERDllExport extern RTIBool 
    ServoControlPluginSupport_copy_data(
        ServoControl *out,
        const ServoControl *in);

    NDDSUSERDllExport extern void 
    ServoControlPluginSupport_destroy_data_w_params(
        ServoControl *sample,
        const struct DDS_TypeDeallocationParams_t * dealloc_params);

    NDDSUSERDllExport extern void 
    ServoControlPluginSupport_destroy_data_ex(
        ServoControl *sample,RTIBool deallocate_pointers);

    NDDSUSERDllExport extern void 
    ServoControlPluginSupport_destroy_data(
        ServoControl *sample);

    NDDSUSERDllExport extern void 
    ServoControlPluginSupport_print_data(
        const ServoControl *sample,
        const char *desc,
        unsigned int indent);

    /* ----------------------------------------------------------------------------
    Callback functions:
    * ---------------------------------------------------------------------------- */

    NDDSUSERDllExport extern PRESTypePluginParticipantData 
    ServoControlPlugin_on_participant_attached(
        void *registration_data, 
        const struct PRESTypePluginParticipantInfo *participant_info,
        RTIBool top_level_registration, 
        void *container_plugin_context,
        RTICdrTypeCode *typeCode);

    NDDSUSERDllExport extern void 
    ServoControlPlugin_on_participant_detached(
        PRESTypePluginParticipantData participant_data);

    NDDSUSERDllExport extern PRESTypePluginEndpointData 
    ServoControlPlugin_on_endpoint_attached(
        PRESTypePluginParticipantData participant_data,
        const struct PRESTypePluginEndpointInfo *endpoint_info,
        RTIBool top_level_registration, 
        void *container_plugin_context);

    NDDSUSERDllExport extern void 
    ServoControlPlugin_on_endpoint_detached(
        PRESTypePluginEndpointData endpoint_data);

    NDDSUSERDllExport extern void    
    ServoControlPlugin_return_sample(
        PRESTypePluginEndpointData endpoint_data,
        ServoControl *sample,
        void *handle);    

    NDDSUSERDllExport extern RTIBool 
    ServoControlPlugin_copy_sample(
        PRESTypePluginEndpointData endpoint_data,
        ServoControl *out,
        const ServoControl *in);

    /* ----------------------------------------------------------------------------
    (De)Serialize functions:
    * ------------------------------------------------------------------------- */

    NDDSUSERDllExport extern RTIBool
    ServoControlPlugin_serialize_to_cdr_buffer(
        char * buffer,
        unsigned int * length,
        const ServoControl *sample); 

    NDDSUSERDllExport extern RTIBool
    ServoControlPlugin_serialize_to_cdr_buffer_ex(
        char *buffer,
        unsigned int *length,
        const ServoControl *sample,
        DDS_DataRepresentationId_t representation);

    NDDSUSERDllExport extern RTIBool
    ServoControlPlugin_deserialize_from_cdr_buffer(
        ServoControl *sample,
        const char * buffer,
        unsigned int length);    
    #ifndef NDDS_STANDALONE_TYPE
    NDDSUSERDllExport extern DDS_ReturnCode_t
    ServoControlPlugin_data_to_string(
        const ServoControl *sample,
        char *str,
        DDS_UnsignedLong *str_size, 
        const struct DDS_PrintFormatProperty *property);    
    #endif

    NDDSUSERDllExport extern unsigned int 
    ServoControlPlugin_get_serialized_sample_max_size(
        PRESTypePluginEndpointData endpoint_data,
        RTIBool include_encapsulation,
        RTIEncapsulationId encapsulation_id,
        unsigned int current_alignment);

    /* --------------------------------------------------------------------------------------
    Key Management functions:
    * -------------------------------------------------------------------------------------- */
    NDDSUSERDllExport extern PRESTypePluginKeyKind 
    ServoControlPlugin_get_key_kind(void);

    NDDSUSERDllExport extern unsigned int 
    ServoControlPlugin_get_serialized_key_max_size(
        PRESTypePluginEndpointData endpoint_data,
        RTIBool include_encapsulation,
        RTIEncapsulationId encapsulation_id,
        unsigned int current_alignment);

    NDDSUSERDllExport extern unsigned int 
    ServoControlPlugin_get_serialized_key_max_size_for_keyhash(
        PRESTypePluginEndpointData endpoint_data,
        RTIEncapsulationId encapsulation_id,
        unsigned int current_alignment);

    NDDSUSERDllExport extern RTIBool 
    ServoControlPlugin_deserialize_key(
        PRESTypePluginEndpointData endpoint_data,
        ServoControl ** sample,
        RTIBool * drop_sample,
        struct RTICdrStream *stream,
        RTIBool deserialize_encapsulation,
        RTIBool deserialize_key,
        void *endpoint_plugin_qos);

    NDDSUSERDllExport extern
    struct RTIXCdrInterpreterPrograms * ServoControlPlugin_get_programs(void);

    /* Plugin Functions */
    NDDSUSERDllExport extern struct PRESTypePlugin*
    ServoControlPlugin_new(void);

    NDDSUSERDllExport extern void
    ServoControlPlugin_delete(struct PRESTypePlugin *);

}

#if (defined(RTI_WIN32) || defined (RTI_WINCE) || defined(RTI_INTIME)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif /* ServoControlPlugin_1848045218_h */

