
// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros();


// Connect to the ROSbridge server
function connectToServer()
{
    //var server_address = 'ws://artica-magabot:9090';
    var server_address = 'ws://sscpc0.ist.utl.pt:28090';
    //var server_address = $('#text_server_address').val();
    // Create a connection to the rosbridge WebSocket server.
    ros.connect(server_address);
    logData('Connecting to "'+server_address+'"', 'message_normal');
}

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error)
{
    logData('Error establishing the connection!', 'message_error');
});

// Callback for when the connection is established.
ros.on('connection', function()
{
    logData('Connection established!');
});


// Callback for when the connection is closed.
ros.on('close', function()
{
    logData('Connection closed.', 'message_normal');
});


// --------------------------------------------------------------------------------------
// Topic for the interface to send the teleop messages
var topic_teleop = new ROSLIB.Topic({
    ros : ros,
    name : '/teleop',
    messageType : 'std_msgs/String'
});

// Publishes a string command on a specific topic
function publishStringCommand(topic, command)
{
    logData('Sending to topic:' +topic.name +' command:'+ command , 'message_ui');
    var com = new ROSLIB.Message(
    {
        data: command
    });

    topic.publish(com);
}



// --------------------------------------------------------------------------------------
// Topic to receive Odometry data from the robot
var listener_odom = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
});

// This function is a callback for when a message is received on this topic
listener_odom.subscribe(function(message)
{
    //console.log('Received message on ' + listener_odom.name + ': ' + message);

    $('#td_linear_velocity_X').html(message.twist.twist.linear.x.toFixed(3));
    $('#td_linear_velocity_Y').html(message.twist.twist.linear.y.toFixed(3));
    $('#td_linear_velocity_Z').html(message.twist.twist.linear.z.toFixed(3));

    $('#td_angular_velocity_X').html(message.twist.twist.angular.x.toFixed(3));
    $('#td_angular_velocity_Y').html(message.twist.twist.angular.y.toFixed(3));
    $('#td_angular_velocity_Z').html(message.twist.twist.angular.z.toFixed(3));

    $('#td_position_X').html(message.pose.pose.position.x.toFixed(3));
    $('#td_position_Y').html(message.pose.pose.position.y.toFixed(3));
    $('#td_position_Z').html(message.pose.pose.position.z.toFixed(3));
});



// --------------------------------------------------------------------------------------
// Topic to receive sensor data from the robot (ir sensors and sonars)
var listener_status = new ROSLIB.Topic({
    ros : ros,
    name : '/magabot/status',
    messageType : 'magabot/Status'
});

// This function is a callback for when a message is received on this topic
listener_status.subscribe(function(message)
{
    //console.log('Received message on ' + listener_status.name + ': ' + message);
    //console.log( message );

    // Changes the background color of each bumper depending of the state
    $('#div_bumper_left').css('background-color', (message.bumpers[0]? '#888888': 'transparent' ) );
    $('#div_bumper_right').css('background-color', (message.bumpers[1]? '#888888': 'transparent' ) );

    // Draws the 3 linear meters from the IR sensors in the assigned canvas
    fillLinearMeter( 'canvas_ir_sensor_0', message.ir[0] / 1023);
    fillLinearMeter( 'canvas_ir_sensor_1', message.ir[1] / 1023);
    fillLinearMeter( 'canvas_ir_sensor_2', message.ir[2] / 1023);

    drawSonars('canvas_sonars', [ message.sonars[0],  message.sonars[1],  message.sonars[2], message.sonars[3], message.sonars[3]]);
});



// --------------------------------------------------------------------------------------
// Topic to receive log info messages
var listener_log_info = new ROSLIB.Topic({
    ros : ros,
    name : '/magabot/loginfo',
    messageType : 'std_msgs/String'
});

// This function is a callback for when a message is received on this topic
listener_log_info.subscribe(function(message)
{
    logDataROS(message, 'message_ui');
});



// --------------------------------------------------------------------------------------
// Topic to receive log error messages
var listener_log_err = new ROSLIB.Topic({
    ros : ros,
    name : '/magabot/logerr',
    messageType : 'std_msgs/String'
});

// This function is a callback for when a message is received on this topic
listener_log_err.subscribe(function(message)
{
    logDataROS(message, 'message_error');

});



var listener_laser_readings = new ROSLIB.Topic({
    ros : ros,
    name : '/laser_xy',
    messageType : 'geometry_msgs/PoseArray'
});

// This function is a callback for when a message is received on this topic
listener_laser_readings.subscribe(function(message)
{
    //logDataROS(message, 'message_ui');
    drawLaserReadings(message);
});



// --------------------------------------------------------------------------------------
// Service to start recording data in the bag
var service_start_bag = new ROSLIB.Service({
    ros : ros,
    name : '/start_rosbag',
    serviceType : 'magabot/StartRosbag'
});

// Calls the service that starts saving data in the bag,
// reading the configuration data on the topics to save from the interface
function startRecordingBag()
{
    logData("Start bag file recording");
    // Create a variable with the desired parameters and their settings from the UI
    var data = {};
    for (var i=0; i<topic_list.length; i++)
    {
        data[topic_list[i].id] = $("#checkbox_"+topic_list[i].id).is(':checked') ? "T" : "F";
        console.log(data)
    }

    //console.log(data);

    // Create a new service request with the previous data and send it
    var request = new ROSLIB.ServiceRequest(data);
    service_start_bag.callService(request, function(result) {
        console.log("Result for service call on " + service_start_bag.name + ": " + result.status);
    });
}


// --------------------------------------------------------------------------------------
// Service to stop recording data in the bag
var service_stop_bag = new ROSLIB.Service({
    ros : ros,
    name : '/stop_rosbag',
    serviceType : 'magabot/StopRosbag'
});

// Calls the service that stops saving data in the bag
function stopRecordingBag()
{
    logData("Stop bag file recording");
    // Create a new service request with the previous data and send it
    var request = new ROSLIB.ServiceRequest();
    service_stop_bag.callService(request, function(result) {
        console.log("Result for service call on " + service_stop_bag.name + ": " + result.status);
    });
}


// --------------------------------------------------------------------------------------
// Service to start running the uploaded program
var service_start_exec = new ROSLIB.Service({
    ros : ros,
    name : '/start_code_exec',
    serviceType : 'magabot/StartCodeExec'
});

// Calls the service that stops saving data in the bag
function startCodeExec()
{
    logData("Start executing the code");
    // Create a new service request with the previous data and send it
    var request = new ROSLIB.ServiceRequest();
    service_start_exec.callService(request, function(result) {
        console.log("Result for service call on " + service_start_exec.name + ": " + result.status);
    });
}


// --------------------------------------------------------------------------------------
// Service to stop running the uploaded program
var service_stop_exec = new ROSLIB.Service({
    ros : ros,
    name : '/stop_code_exec',
    serviceType : 'magabot/StopCodeExec'
});

// Calls the service that stops saving data in the bag
function stopCodeExec()
{
    logData("Stop executing the code");
    // Create a new service request with the previous data and send it
    var request = new ROSLIB.ServiceRequest();
    service_stop_exec.callService(request, function(result) {
        console.log("Result for service call on " + service_stop_exec.name + ": " + result.status);
    });
}
