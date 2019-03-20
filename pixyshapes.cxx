/* Pixyshapes.cxx

Shapes Live main program updated for use w/ Pixycam2

*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <signal.h>
#include <string.h>
#include "libpixyusb2.h"

#include "ShapeType.h"
#include "ShapeTypeSupport.h"

#include "ServoControlPlugin.h"
#include "ServoControlSupport.h"

#define TRACKING_ENABLED

#ifdef TRACKING_ENABLED
#include "ServoControl.h"
#include "ServoControlSupport.h"
#endif

#include "ndds/ndds_cpp.h"

//===================================
// PIXY STUFF
Pixy2 pixy;

#if 0
#define BLOCK_BUFFER_SIZE  25
// Pixy Block buffer //
struct Block pixyBlocks[BLOCK_BUFFER_SIZE];
#endif

static bool run_flag = true;
static bool got_matched_subscriber = false;
static bool got_matched_publisher = false;

#define NUM_SIGS 7

static bool sigEnabled[NUM_SIGS];

// These are DDS Shapes topic names used for each PixyCam signature.
// The PixyCam signatures have the same names and range from #1 (Red) to #7 (Violet)
const char *sigName[] = {
    "RED",
    "ORANGE",
    "YELLOW",
    "GREEN",
    "CYAN",
    "BLUE",
    "PURPLE"
};

#define SIG_RED  1
#define SIG_ORANGE 2
#define SIG_YELLOW 3
#define SIG_GREEN 4
#define SIG_CYAN 5
#define SIG_BLUE 6
#define SIG_VIOLET 7

#define S0_LOWER_LIMIT -200
#define S0_UPPER_LIMIT 200
#define S1_LOWER_LIMIT -200
#define S1_UPPER_LIMIT 200
#define SERVO_FREQUENCY_HZ 60
#define PAN_P_GAIN 350
#define PAN_D_GAIN 600
#define TILT_P_GAIN 500
#define TILT_D_GAIN 700

#define SHAPE_X_MIN 0
#define SHAPE_X_MAX 222
#define SHAPE_Y_MIN 0
#define SHAPE_Y_MAX 252

#define PIXY_MAX_X 319
#define PIXY_MAX_Y 199

// PID control parameters //
#define PAN_PROPORTIONAL_GAIN     400	// 350
#define PAN_DERIVATIVE_GAIN       300	// 600
#define TILT_PROPORTIONAL_GAIN    500	// 500
#define TILT_DERIVATIVE_GAIN      400	// 700

#ifdef TRACKING_ENABLED
//-------------------------------------------------------------------
// We'll need one of these for pan, and one for tilt.  Holds
// variables for running the tracking algorithm
//-------------------------------------------------------------------
struct Gimbal {
  int32_t position;
  int32_t previous_error;
  int32_t proportional_gain;
  int32_t derivative_gain;
};

struct Gimbal pan;
struct Gimbal tilt;
#endif

//-------------------------------------------------------------------
// Local prototypes
//-------------------------------------------------------------------
void handle_SIGINT(int unused);
static int publisher_shutdown(DDSDomainParticipant *participant);
extern "C" int publisher_main(int domainId, int sample_count);
int main(int argc, char *argv[]);

#ifdef TRACKING_ENABLED
void initialize_gimbals(void);
void gimbal_update(struct Gimbal *  gimbal, int32_t error);
//-------------------------------------------------------------------
// Setup the gimbal control structures
//-------------------------------------------------------------------
void initialize_gimbals(void)
{
	pan.position = PIXY_RCS_CENTER_POS;
	pan.previous_error     = 0x80000000L;
	pan.proportional_gain  = PAN_PROPORTIONAL_GAIN;
	pan.derivative_gain    = PAN_DERIVATIVE_GAIN;
	tilt.position          = PIXY_RCS_CENTER_POS;
	tilt.previous_error    = 0x80000000L;
	tilt.proportional_gain = TILT_PROPORTIONAL_GAIN;
	tilt.derivative_gain   = TILT_DERIVATIVE_GAIN;
}

//-------------------------------------------------------------------
// This is the control loop for each axis of the cam control (pan/tilt)
//-------------------------------------------------------------------
void gimbal_update(struct Gimbal *  gimbal, int32_t error)
{
	long int velocity;
	int32_t  error_delta;
	int32_t  P_gain;
	int32_t  D_gain;

	if(gimbal->previous_error != 0x80000000L)
	{
		error_delta = error - gimbal->previous_error;
		P_gain      = gimbal->proportional_gain;
		D_gain      = gimbal->derivative_gain;

		/* Using the proportional and derivative gain for the gimbal,
	   	   calculate the change to the position.  */
		velocity = (error * P_gain + error_delta * D_gain) >> 10;

		gimbal->position += velocity;

		if (gimbal->position > PIXY_RCS_MAX_POS)
		{
			gimbal->position = PIXY_RCS_MAX_POS;
		} else if (gimbal->position < PIXY_RCS_MIN_POS)
		{
			gimbal->position = PIXY_RCS_MIN_POS;
		}
	}

  gimbal->previous_error = error;
}
#endif //#ifdef

//-------------------------------------------------------------------
// Handle Ctrl-C
//-------------------------------------------------------------------
void handle_SIGINT(int unused)
{
  // On CTRL+C - abort! //

  run_flag = false;
}

//-------------------------------------------------------------------
// Listener class for ShapeType writer
//-------------------------------------------------------------------
class ShapeTypeExtendedListener : public DDSDataWriterListener {
  public:
    virtual void on_offered_deadline_missed(
        DDSDataWriter * /*reader*/,
        const DDS_OfferedDeadlineMissedStatus& /*status*/) {}

    virtual void on_liveliness_lost(
        DDSDataWriter *,
        const DDS_LivelinessLostStatus &) {}

    virtual void on_offered_incompatible_qos(
        DDSDataWriter *,
        const DDS_OfferedIncompatibleQosStatus &) {}

    virtual void on_reliable_writer_cache_changed(
        DDSDataWriter *,
        const DDS_ReliableWriterCacheChangedStatus &) {}

    virtual void on_reliable_reader_activity_changed(
        DDSDataWriter *,
        const DDS_ReliableReaderActivityChangedStatus &) {}

    virtual void on_instance_replaced (
        DDSDataWriter *,
        const DDS_InstanceHandle_t &) {}

    virtual void on_application_acknowledgment(
        DDSDataWriter *,
        const DDS_AcknowledgmentInfo &) {}

    virtual void on_publication_matched(DDSDataWriter *, const DDS_PublicationMatchedStatus &);

};

//-------------------------------------------------------------------
// Listener function to handle matched subscribers
//-------------------------------------------------------------------
void ShapeTypeExtendedListener::on_publication_matched(DDSDataWriter *writer, const DDS_PublicationMatchedStatus &status)
{
	ShapeTypeExtendedDataWriter *ShapeType_writer = NULL;
	DDS_ReturnCode_t retcode;

	ShapeType_writer = ShapeTypeExtendedDataWriter::narrow(writer);
	if (NULL == ShapeType_writer)
	{
		printf("DataWriter narrow error\n");
		return;
	}

	printf("\n");
    printf("Subs: %d %d\n", status.current_count, status.current_count_change);

#if 0
	printf("Match status\n");
	printf("----------------------------\n");
	printf("current_count       : %ld\n", status.current_count);
	printf("current_count_change: %ld\n", status.current_count_change);
	printf("current_count_peak  : %ld\n", status.current_count_peak);
	printf("total_count         : %ld\n", status.total_count);
	printf("total_count_change  : %ld\n", status.total_count_change);
	printf("\n");
#endif

    if (status.current_count > 0)
    {
        got_matched_subscriber = true;
    } else
    {
        got_matched_subscriber = false;
    }
}

//-------------------------------------------------------------------
// Listener class for data reader
//-------------------------------------------------------------------
class ServoControlListener : public DDSDataReaderListener
{
public:
    virtual void on_requested_deadline_missed(
        DDSDataReader* /*reader*/,
        const DDS_RequestedDeadlineMissedStatus& /*status*/) {}

    virtual void on_requested_incompatible_qos(
        DDSDataReader* /*reader*/,
        const DDS_RequestedIncompatibleQosStatus& /*status*/) {}

    virtual void on_sample_rejected(
        DDSDataReader* /*reader*/,
        const DDS_SampleRejectedStatus& /*status*/) {}

    virtual void on_liveliness_changed(
        DDSDataReader* /*reader*/,
        const DDS_LivelinessChangedStatus& /*status*/) {}

    virtual void on_sample_lost(
        DDSDataReader* /*reader*/,
        const DDS_SampleLostStatus& /*status*/) {}

    virtual void on_subscription_matched(
        DDSDataReader* /*reader*/,
        const DDS_SubscriptionMatchedStatus & /*status*/);

    virtual void on_data_available(DDSDataReader* reader){}

};

void ServoControlListener::on_subscription_matched(DDSDataReader *reader, const DDS_SubscriptionMatchedStatus &status)
{
	ServoControlDataReader *servo_reader = NULL;
	DDS_ReturnCode_t retcode;

	servo_reader = ServoControlDataReader::narrow(reader);
	if (NULL == servo_reader) return;

	printf("\n");
	printf("Pubs: %d %d\n", status.current_count, status.current_count_change);
	if (status.current_count > 0)
		got_matched_publisher = true;
	else
		got_matched_publisher = false;

	return;
}


//-------------------------------------------------------------------
// Do an orderly shutdown
//-------------------------------------------------------------------
static int publisher_shutdown(DDSDomainParticipant *participant)
{
    DDS_ReturnCode_t retcode;
    int status = 0;

    if (participant != NULL) {
        retcode = participant->delete_contained_entities();
        if (retcode != DDS_RETCODE_OK) {
            printf("delete_contained_entities error %d\n", retcode);
            status = -1;
        }

        retcode = DDSTheParticipantFactory->delete_participant(participant);
        if (retcode != DDS_RETCODE_OK) {
            printf("delete_participant error %d\n", retcode);
            status = -1;
        }
    }
    retcode = DDSDomainParticipantFactory::finalize_instance();
    if (retcode != DDS_RETCODE_OK) {
        printf("finalize_instance error %d\n", retcode);
        status = -1;
    }
    return status;
}

//-------------------------------------------------------------------
// Main loop
//-------------------------------------------------------------------
extern "C" int publisher_main(int domainId, int sample_count)
{
    DDSDomainParticipant *participant = NULL;
    DDSPublisher *publisher = NULL;
    DDSTopic *topic = NULL;
    DDSDataWriter *writer = NULL;
    ShapeTypeExtendedDataWriter * ShapeTypeExtended_writer = NULL;
    ShapeTypeExtended *trackedObject[NUM_SIGS];

#ifdef TRACKING_ENABLED
    // Entities for servo control
    DDSSubscriber *subscriber = NULL;
    DDSTopic *servo_topic = NULL;
    DDSDataReader *reader = NULL;
    const char *servo_type_name = NULL;
    ServoControl servo_control;
    DDS_SampleInfo servo_info;
#endif // TRACKING_ENABLED

    DDS_ReturnCode_t retcode;
    DDS_InstanceHandle_t objectHandle[NUM_SIGS];
    const char *type_name = NULL;
    int count = 0;
    //DDS_Duration_t send_period = {0,500000000};

    //===================================
    // PIXY STUFF
    int pixyFrame = 0;
    int pixyIndex;
    int pixyBlocksCopied;
    char pixyBuf[128];
    bool objectChanged[NUM_SIGS];
    int sigNum;
    // END PIXY STUFF
    //===================================

    /* To customize participant QoS, use
    the configuration file USER_QOS_PROFILES.xml */
    participant = DDSTheParticipantFactory->create_participant(
        domainId, DDS_PARTICIPANT_QOS_DEFAULT,
        NULL /* listener */, DDS_STATUS_MASK_NONE);
    if (participant == NULL) {
        printf("create_participant error\n");
        publisher_shutdown(participant);
        return -1;
    }

    /* To customize publisher QoS, use
    the configuration file USER_QOS_PROFILES.xml */
    publisher = participant->create_publisher(
        DDS_PUBLISHER_QOS_DEFAULT, NULL /* listener */, DDS_STATUS_MASK_NONE);
    if (publisher == NULL) {
        printf("create_publisher error\n");
        publisher_shutdown(participant);
        return -1;
    }

    /* Register type before creating topic */
    type_name = ShapeTypeExtendedTypeSupport::get_type_name();
    retcode = ShapeTypeExtendedTypeSupport::register_type(
        participant, type_name);
    if (retcode != DDS_RETCODE_OK) {
        printf("register_type error %d\n", retcode);
        publisher_shutdown(participant);
        return -1;
    }

    /* To customize topic QoS, use
    the configuration file USER_QOS_PROFILES.xml */
    topic = participant->create_topic(
        "Circle",
        type_name, DDS_TOPIC_QOS_DEFAULT, NULL /* listener */,
        DDS_STATUS_MASK_NONE);
    if (topic == NULL) {
        printf("create_topic error\n");
        publisher_shutdown(participant);
        return -1;
    }

    ShapeTypeExtendedListener *writer_listener = new ShapeTypeExtendedListener();
    if (NULL == writer_listener)
    {
        printf("Could not create writer_listener!\r\n");
        return 1;
    }

    /* To customize data writer QoS, use
    the configuration file USER_QOS_PROFILES.xml */
    writer = publisher->create_datawriter(
        topic, DDS_DATAWRITER_QOS_DEFAULT, writer_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (writer == NULL) {
        printf("create_datawriter error\n");
        publisher_shutdown(participant);
        return -1;
    }
    ShapeTypeExtended_writer = ShapeTypeExtendedDataWriter::narrow(writer);
    if (ShapeTypeExtended_writer == NULL) {
        printf("DataWriter narrow error\n");
        publisher_shutdown(participant);
        return -1;
    }

    for (count = 0; count < NUM_SIGS; count++)
    {
        // Create a holder for the object
        objectHandle[count] = DDS_HANDLE_NIL;
        trackedObject[count] = ShapeTypeExtendedTypeSupport::create_data();
        if (trackedObject[count] == NULL) {
            printf("ShapeTypeExtendedTypeSupport::create_data_error\n");
            publisher_shutdown(participant);
            return -1;
        }
        // Initialize the object
        strcpy(trackedObject[count]->color,sigName[count]);
        trackedObject[count]->x = 50;
        trackedObject[count]->y = 50;
        trackedObject[count]->shapesize = 20;

        // Set its changed flag to false
        objectChanged[count] = false;
    }
    //sigEnabled[SIG_BLUE - 1] = true;
    //sigEnabled[SIG_GREEN -1] = true;

#ifdef TRACKING_ENABLED
    //---------------------------------------------------------------
    // Create DDS entities for servo control
    //---------------------------------------------------------------
    subscriber = participant->create_subscriber(DDS_SUBSCRIBER_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (subscriber == NULL)
    {
        printf("create_subscriber error\n");
        publisher_shutdown(participant);
        return -1;
    }

    // Register the type and create a topic
	servo_type_name = ServoControlTypeSupport::get_type_name();
	retcode = ServoControlTypeSupport::register_type(participant, servo_type_name);
	servo_topic = participant->create_topic(DEFAULT_CAM_CONTROL_TOPIC_NAME, servo_type_name, DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if ((retcode != DDS_RETCODE_OK) || (servo_topic == NULL))
    {
        printf("register servo type/topic error\n");
        publisher_shutdown(participant);
        return -1;
    }

    ServoControlListener *servo_listener = new ServoControlListener();

    // Create a data reader and narrow it to a type-specific one
	reader = participant->create_datareader(servo_topic, DDS_DATAREADER_QOS_DEFAULT, servo_listener, DDS_STATUS_MASK_ALL);
	ServoControlDataReader *servo_reader = ServoControlDataReader::narrow(reader);
	if ((reader == NULL) || (servo_reader == NULL))
    {
        printf("create servo reader error\n");
        publisher_shutdown(participant);
        return -1;
    }

    ServoControl_initialize(&servo_control);

#endif // TRACKING_ENABLED


    /* For a data type that has a key, if the same instance is going to be
    written multiple times, initialize the key here
    and register the keyed instance prior to writing */

#if 0
    orangeHandle = ShapeTypeExtended_writer->register_instance(*orangeObject);
#endif

    printf("Signature:\n");
    for (count = 0; count < NUM_SIGS; count++)
    {
        printf("%s:",trackedObject[count]->color);
        if(sigEnabled[count])
        {
            printf("On\n");
        }
        else
        {
            printf("Off\n");
        }
    }

    /* Main loop */
    while (run_flag) {

        // Read a frame from the camera
        pixy.ccc.getBlocks(false, 0x7F, 255);

        // For each detected block, update the appropriate Circle
        for (int block = 0; block < pixy.ccc.numBlocks; block++)
        {
            sigNum = pixy.ccc.blocks[block].m_signature - 1;
            if (sigEnabled[sigNum] == true)
            {
                // Scale camera pixels to Shapes demo pixels
                int x = (pixy.ccc.blocks[block].m_x * SHAPE_X_MAX) / PIXY_MAX_X;
                int y = (pixy.ccc.blocks[block].m_y * SHAPE_Y_MAX) / PIXY_MAX_Y;
                // Update this ShapeType instance
                trackedObject[sigNum]->x = x;
                trackedObject[sigNum]->y = y;
                trackedObject[sigNum]->shapesize = pixy.ccc.blocks[block].m_width;
                objectChanged[sigNum] = true;
            }

        } // for loop

        if (got_matched_subscriber) // Only write data if someone is listening
        {
            // Send DDS updates for any signatures that have new data
            for (sigNum = 0; sigNum < NUM_SIGS; sigNum++)
            {
                if(sigEnabled[sigNum] && objectChanged[sigNum])
                {
                    retcode = ShapeTypeExtended_writer->write(*(trackedObject[sigNum]), objectHandle[sigNum]);
                    if (retcode != DDS_RETCODE_OK)
                    {
                        printf("write error %d\n", retcode);
                    }
                }
                objectChanged[sigNum] = false;
            }
            ShapeTypeExtended_writer->flush();
        } // if (got_matched_subscriber)

        // Now check for inbound servo control data
#ifdef TRACKING_ENABLED
        if (got_matched_publisher)
        {
            retcode = servo_reader->take_next_sample(servo_control, servo_info);
            if ((retcode == DDS_RETCODE_OK) && (servo_info.valid_data == RTI_TRUE))
            {
                // Update the servo positions on the pixy
                //pixy_rcs_set_position(0, servo_control.pan);
                //pixy_rcs_set_position(1, servo_control.tilt);
                pixy.setServos(servo_control.pan, servo_control.tilt);
                printf("P: %d T: %d    \r", servo_control.pan, servo_control.tilt);
            }
        }
#endif

//        NDDSUtility::sleep(send_period);
    }

    for (count = 0; count < NUM_SIGS; count++) {
        retcode = ShapeTypeExtendedTypeSupport::delete_data(trackedObject[count]);
        if (retcode != DDS_RETCODE_OK) {
    	       printf("ShapeTypeExtendedTypeSupport::delete_data_error %d\n", retcode);
        }
    }

    /* Delete all entities */
    return publisher_shutdown(participant);
}

//-------------------------------------------------------------------
// Program entry point
//-------------------------------------------------------------------
int main(int argc, char *argv[])
{
    int domainId = 53;
    int sample_count = 0; /* infinite loop */
    int return_value;

    // Handle control-C
    signal(SIGINT, handle_SIGINT);

    // Show the domain we are publishing on
    printf("DomainID: %d\n", domainId);

    // Initialize Pixycam2
    return_value = pixy.init();
    if (return_value < 0)
    {
        printf("pixy.init(): %d\n ", return_value);
        return(return_value);
    }

    // Request Pixycam2 firmware version
    return_value = pixy.getVersion();
    if (return_value < 0)
    {
        printf("pixy.init(): %d\n ", return_value);
        return(return_value);
    }
    pixy.version->print();
    pixy.changeProg("color_connected_components");

    if (argc > 1)
    {
        for (int count = 1; count < argc; count++)
        {
            for (int sigs = 0; sigs < NUM_SIGS; sigs++)
            {
                if (strcmp(argv[count],sigName[sigs]) == 0)
                {
                    sigEnabled[sigs] = true ;
                }
            }

        }
    }
    else
    {
        // If not colors are enabled on the command line, turn on GREEN circles
        sigEnabled[SIG_GREEN - 1] = true;
    }

    /* Uncomment this to turn on additional logging
    NDDSConfigLogger::get_instance()->
    set_verbosity_by_category(NDDS_CONFIG_LOG_CATEGORY_API,
    NDDS_CONFIG_LOG_VERBOSITY_STATUS_ALL);
    */

    return publisher_main(domainId, sample_count);
}
