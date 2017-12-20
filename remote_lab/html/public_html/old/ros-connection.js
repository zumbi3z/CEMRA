
// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros();

var listener_odom = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
});

var listener_status = new ROSLIB.Topic({
    ros : ros,
    name : '/magabot/status',
    messageType : 'magabot/Status'
});

var listener_action = new ROSLIB.Topic({
    ros : ros,
    name : '/action',
    messageType : 'std_msgs/String'
});

var isConnected = false;

function connectToServer()
{
    var server_address = $('#text_server_address').val();
    // Create a connection to the rosbridge WebSocket server.
    if (!isConnected) {
	ros.connect(server_address);
	logData('\nConnecting to ' + server_address + '...');

	listener_odom.subscribe(listener_odom_cb);
	listener_status.subscribe(listener_status_cb);
	listener_action.subscribe(listener_action_cb);

	/*$.getJSON("https://api.ipify.org?format=jsonp&callback=?",
		  function(json) {
		      logData("My public IP address is: ", json.ip);
		  }
	);*/

	//logData(ip);
    } else {
	logData('\nAlready connected to ' + server_address + '!');
    }
}


function disconnectFromServer()
{
    var server_address = $('#text_server_address').val();
    if (isConnected) {
	publishStringCommand(topic_action, 'stop_teleop');
	logData('\nDisconnecting from ' + server_address + '...');
	ros.close(server_address)
	// logData('Connection closed.');
	isConnected = false;
	listener_odom.unsubscribe();
    	listener_status.unsubscribe();
	listener_action.unsubscribe();
    } else {
	logData('\nNot connected to ' + server_address);
    }
}


// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error)
{
    logData('\nError establishing the connection!');
    isConnected = false;
    reinitialize_status();
});

// Callback for when the connection is established.
ros.on('connection', function()
{
    logData('Connection established!');
    publishStringCommand(topic_action, 'start_teleop');
    isConnected = true;
});


// Callback for when the connection is closed.
ros.on('close', function()
{
    //publishStringCommand(topic_action, 'stop_teleop');
    logData('Connection closed!');
    publishStringCommand(topic_action, 'stop_teleop');
    isConnected = false;
    reinitialize_status();
    listener_odom.unsubscribe();
    listener_status.unsubscribe();
    listener_action.unsubscribe();
});



// Publishing a Topic
// ------------------

// First, we create a Topic object with details of the topic's name and message type.
var topic_teleop = new ROSLIB.Topic({
    ros : ros,
    name : '/teleop',
    messageType : 'std_msgs/String'
});

var topic_action = new ROSLIB.Topic({
    ros : ros,
    name : '/action',
    messageType : 'std_msgs/String'
});


// Publishes a string command on a specific topic
function publishStringCommand(topic, command)
{
    //logData('Sending to topic:' + topic.name + ' command:' + command);
    var com = new ROSLIB.Message(
    {
        data: command
    });

    topic.publish(com);
}


function listener_odom_cb(message) {
    //console.log('Received message on ' + listener_odom.name + ': ' + message);

    $('#td_linear_velocity_X').html(message.twist.twist.linear.x.toFixed(3));
    $('#td_linear_velocity_Y').html(message.twist.twist.linear.y.toFixed(3));
    $('#td_linear_velocity_THETA').html(message.twist.twist.linear.z.toFixed(3));

    $('#td_angular_velocity_X').html(message.twist.twist.angular.x.toFixed(3));
    $('#td_angular_velocity_Y').html(message.twist.twist.angular.y.toFixed(3));
    $('#td_angular_velocity_THETA').html(message.twist.twist.angular.z.toFixed(3));

    $('#td_position_X').html(message.pose.pose.position.x.toFixed(3));
    $('#td_position_Y').html(message.pose.pose.position.y.toFixed(3));

    var q = new THREE.Quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w);
    var euler = new THREE.Euler().setFromQuaternion(q, 'XYZ');
    var heading = euler["_z"];
    $('#td_position_THETA').html(heading.toFixed(3));

    //console.log(message);

    // If desired, we can unsubscribe from the topic as well.
    //listener.unsubscribe();
}


function reinitialize_status() {
	if (!isConnected) {
		$('#div_bumper_left').css('background-color', 'transparent');
		$('#div_bumper_right').css('background-color', 'transparent');

		fillLinearMeter('canvas_ir_sensor_0', 0);
		fillLinearMeter('canvas_ir_sensor_1', 0);
		fillLinearMeter('canvas_ir_sensor_2', 0);
		
		var zero = 0;
	        $('#td_linear_velocity_X').html(zero.toFixed(3));
		$('#td_linear_velocity_Y').html(zero.toFixed(3));
		$('#td_linear_velocity_THETA').html(zero.toFixed(3));

		$('#td_angular_velocity_X').html(zero.toFixed(3));
		$('#td_angular_velocity_Y').html(zero.toFixed(3));
		$('#td_angular_velocity_THETA').html(zero.toFixed(3));

		$('#td_position_X').html(zero.toFixed(3));
		$('#td_position_Y').html(zero.toFixed(3));
		$('#td_position_THETA').html(zero.toFixed(3));
	}
}


function listener_status_cb(message) {
    //console.log('Received message on ' + listener_status.name + ': ' + message);
    //console.log( message );

    // Changes the background color of each bumper depending of the state
    $('#div_bumper_left').css('background-color', (message.bumpers[0]? '#888888': 'transparent' ) );
    $('#div_bumper_right').css('background-color', (message.bumpers[1]? '#888888': 'transparent' ) );

    //console.log(message.bumpers[0]);

 //   fillLinearMeter( 'canvas_bumper_left', (message.bumpers[0]? 1: 0) );

    // Draws the 3 linear meters from the IR sensors in the assigned canvas
    fillLinearMeter( 'canvas_ir_sensor_0', message.ir[0] / 1023);
    fillLinearMeter( 'canvas_ir_sensor_1', message.ir[1] / 1023);
    fillLinearMeter( 'canvas_ir_sensor_2', message.ir[2] / 1023);

    drawSonars('canvas_sonars', [ message.sonars[0],  message.sonars[1],  message.sonars[2], message.sonars[3], message.sonars[3]]);

}

function listener_action_cb(message) {
    if (message.data == 'disconnect') {
	var server_address = $('#text_server_address').val();
	logData("Someone is already connected. Closing connection...");
	ros.close(server_address);
	logData("Connection closed!");
    }
}



// Calling a service
// -----------------

/*
// First, we create a Service client with details of the service's name and service type.
var addTwoIntsClient = new ROSLIB.Service({
    ros : ros,
    name : '/add_two_ints',
    serviceType : 'rospy_tutorials/AddTwoInts'
});

// Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the
// fields defined in the rospy_tutorials AddTwoInts.srv file.
var request = new ROSLIB.ServiceRequest({
    a : 1,
    b : 2
});

// Finally, we call the /add_two_ints service and get back the results in the callback. The result
// is a ROSLIB.ServiceResponse object.
addTwoIntsClient.callService(request, function(result) {
    console.log('Result for service call on ' + addTwoIntsClient.name + ': ' + result.sum);
});

*/

/*
// Setting a param value
// ---------------------

ros.getParams(function(params) {
  console.log(params);
});

// First, we create a Param object with the name of the param.
var maxVelX = new ROSLIB.Param({
    ros : ros,
    name : 'max_vel_y'
});

//Then we set the value of the param, which is sent to the ROS Parameter Server.
maxVelX.set(0.8);
maxVelX.get(function(value) {
    console.log('MAX VAL: ' + value);
});

// Getting a param value
// ---------------------

var favoriteColor = new ROSLIB.Param({
    ros : ros,
    name : 'favorite_color'
});

favoriteColor.set('red');
favoriteColor.get(function(value) {
  console.log('My robot\'s favorite color is ' + value);
});

*/
