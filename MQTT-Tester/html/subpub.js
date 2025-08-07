Header_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Header_pub.publish(msg);
});

Int8MultiArray_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Int8MultiArray_pub.publish(msg);
});

UInt8MultiArray_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    UInt8MultiArray_pub.publish(msg);
});

Int16MultiArray_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Int16MultiArray_pub.publish(msg);
});

UInt16MultiArray_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    UInt16MultiArray_pub.publish(msg);
});

Int32MultiArray_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Int32MultiArray_pub.publish(msg);
});

UInt32MultiArray_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    UInt32MultiArray_pub.publish(msg);
});

Transform_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Transform_pub.publish(msg);
});

TransformStamped_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TransformStamped_pub.publish(msg);
});

Vector3_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Vector3_pub.publish(msg);
});

Quaternion_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Quaternion_pub.publish(msg);
});

Joy_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Joy_pub.publish(msg);
});

ParentBind_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    ParentBind_pub.publish(msg);
});

BoundingBox_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    BoundingBox_pub.publish(msg);
});

Detection_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Detection_pub.publish(msg);
});

DriveGains_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    DriveGains_pub.publish(msg);
});

MotorParams_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    MotorParams_pub.publish(msg);
});

MotorStatus_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    MotorStatus_pub.publish(msg);
});

Route_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Route_pub.publish(msg);
});

TriorbAlignPos3_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbAlignPos3_pub.publish(msg);
});

TriorbPos3_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbPos3_pub.publish(msg);
});

TriorbPos3Stamped_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbPos3Stamped_pub.publish(msg);
});

TriorbRunPos3_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbRunPos3_pub.publish(msg);
});

TriorbRunResult_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbRunResult_pub.publish(msg);
});

TriorbRunResultStamped_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbRunResultStamped_pub.publish(msg);
});

TriorbRunState_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbRunState_pub.publish(msg);
});

TriorbRunSetting_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbRunSetting_pub.publish(msg);
});

TriorbRunVel3_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbRunVel3_pub.publish(msg);
});

TriorbRunVel3Stamped_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbRunVel3Stamped_pub.publish(msg);
});

TriorbSetPath_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbSetPath_pub.publish(msg);
});

TriorbSetPos3_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbSetPos3_pub.publish(msg);
});

TriorbSpeed_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbSpeed_pub.publish(msg);
});

TriorbVel3_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    TriorbVel3_pub.publish(msg);
});

CameraDevice_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    CameraDevice_pub.publish(msg);
});

DistanceSensor_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    DistanceSensor_pub.publish(msg);
});

ImuSensor_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    ImuSensor_pub.publish(msg);
});

Obstacles_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    Obstacles_pub.publish(msg);
});

CamerasLandmarkInfo_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    CamerasLandmarkInfo_pub.publish(msg);
});

CamerasPose_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    CamerasPose_pub.publish(msg);
});

PointArrayStamped_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    PointArrayStamped_pub.publish(msg);
});

PoseDevStamped_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    PoseDevStamped_pub.publish(msg);
});

UInt32MultiArrayStamped_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    UInt32MultiArrayStamped_pub.publish(msg);
});

XyArrayStamped_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    XyArrayStamped_pub.publish(msg);
});

ClockSync_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    ClockSync_pub.publish(msg);
});

HostStatus_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    HostStatus_pub.publish(msg);
});

NodeInfo_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    NodeInfo_pub.publish(msg);
});

RobotError_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    RobotError_pub.publish(msg);
});

RobotStatus_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    RobotStatus_pub.publish(msg);
});

SettingIPv4_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    SettingIPv4_pub.publish(msg);
});

SettingROS_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    SettingROS_pub.publish(msg);
});

SettingSSID_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    SettingSSID_pub.publish(msg);
});

StringList_sub.subscribe(function(msg) {
    let elm = document.getElementById('mqtt-sub');
    elm.innerHTML = JSON.stringify(msg, null, "\t");
    StringList_pub.publish(msg);
});

