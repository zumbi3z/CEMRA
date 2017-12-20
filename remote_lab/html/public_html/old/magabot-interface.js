var MAX_SONAR_VALUE = 80;

// Logs a string of data to the console and the log textbox on the page
function logData( str )
{
    var status_log = document.getElementById("status_log");
    if (status_log.value == "") {
	status_log.value = str + '\n';
    } else {
	status_log.value = status_log.value + str + '\n';
    }
    //console.log(str);
    status_log.scrollTop = status_log.scrollHeight;
    
}


function clearSystemLog (){
    var status_log = document.getElementById("status_log");
    status_log.value = "";
}

function drawInterface()
{
    // Create all the control buttons inside the div
    html = '';

    // Create all the directional buttons
   for (var i=0; i<4; i++)
    {
        html += "<img id='button_dir_"+i+"' class='button_dir' src='img/btn_arrow.svg' "+
          "style='position:absolute;'></img>";
    }
    html += "<img id='button_stop' class='button_dir' src='img/btn_stop.svg' "+
      "style='position:absolute;'></img>";


    html += "<img id='button_increase_linear' class='button_adjust' src='img/btn_plus.svg' "+
        "style='position:absolute;'></img>"+
        "<img id='button_decrease_linear' class='button_adjust' src='img/btn_minus.svg' "+
        "style='position:absolute;'></img>"+
        "<img id='button_increase_radial' class='button_adjust' src='img/btn_plus.svg' "+
        "style='position:absolute;'></img>"+
        "<img id='button_decrease_radial' class='button_adjust' src='img/btn_minus.svg' "+
        "style='position:absolute;'></img>"+

        "<img id='img_linear_control' class='button_adjust' src='img/icon_linear.svg' "+
        "style='position:absolute;'></img>"+
        "<img id='img_radial_control' class='button_adjust' src='img/icon_rotation.svg' "+
        "style='position:absolute;'></img>";

    $('#div_control_buttons').html(html);

    // Set the callback functions for the different buttons
    $('.button_dir').on('click', pressButton);
    $('.button_adjust').on('click', pressButton);
    $('#button_disconnect').click( disconnectFromServer );
    $('#button_clearlog').click( clearSystemLog );

    resizeInterface();
    //reinitialize_status();
}

/*var ctx = document.getElementById('test_canvas').getContext('2d');
var img = new Image();
img.src = "http://localhost:8080/stream?topic=axis/image_raw1";
function stream1 (){
	//ctx = document.getElementById('test_canvas').getContext('2d');
	//ctx = $('#test_canvas')[0].getContext('2d');
	img = new Image();
	img.src = "http://localhost:8080/stream?topic=axis/image_raw1";
	img.onload = function() {
		//ctx.drawImage(img, 0, 0);
		console.log('Image loaded...')
	};
}*/

function initialize_images(){
	img = new Image();
	img2 = new Image();
	//img.src = "http://127.0.0.1:10000/stream?topic=/axis/image_raw1";
	//img2.src = "http://127.0.0.2:10001/stream?topic=axis/image_raw2";
	img.src = "http://sscpc0.ist.utl.pt:28001/stream?topic=axis/image_raw1";
	img2.src = "http://sscpc0.ist.utl.pt:28001/stream?topic=axis/image_raw2";
}

function refreshCanvas(){
	ctx = document.getElementById('test_canvas').getContext('2d');
	ctx.drawImage(img, 0, 0);
}

function refreshCanvas2(){
	ctx2 = document.getElementById('test_canvas2').getContext('2d');
	ctx2.drawImage(img2, 0, 0);
}

window.setInterval("refreshCanvas()", 1);
window.setInterval("refreshCanvas2()", 1);

function gcd (a, b) {
    return (b == 0) ? a : gcd (b, a%b);
}

function resizeInterface()
{
    var window_width = $(window).width()
    var window_height = $(window).height()

    var available_w = screen.availWidth;
    var available_h = screen.availHeight;
    //var r = gcd (window_width, window_height);
    //var a1 = window_width/r;
    //var a2 = window_height/r;
    //var a3 = "Aspect = ";
    //var aspect = a3.concat(a1.toString(), ":", a2.toString());
    //logData(window_width)
    //logData(window_height)
    //logData(w)
    //logData(h)
    //logData(r)
    //logData(aspect)

    // Resize the video div to a 4/3 ratio
    $('#div_video_1').css('height', $('#div_video_1').width() * available_h/available_w);
    //$('#mjpeg1').css('height', $('#mjpeg1').width()*0.75);
    $('#test_canvas').css('height', $('#test_canvas').width() * available_h/available_w);
    $('#div_video_2').css('height', $('#div_video_2').width() * available_h/available_w);
    $('#test_canvas2').css('height', $('#test_canvas2').width() * available_h/available_w);

    // Resize the buttons div to a 4/3 ratio
    //$('#div_control_buttons').css('height', $('#div_control_buttons').width() * 499/339);
    $('#div_control_buttons').css('height', $('#div_control_buttons').width() * available_w/available_h);

    var log_height = $('#div_log').height() - $('#status_log').offset().top - $('#status_log').offset().left;

    //console.log('status_log.left():'+log_height);

    // Resize the extbow with the log to use all the available height
    $('#status_log').css('height',  log_height+'px' );


    //console.log('NHEC:' + ($('#div_log').css('height') - $('#status_log').offset().top ));

    //console.log('div_log:' + $('#div_log').height() )  ;
    //console.log('status_log:' + $('#status_log').offset().top );

    $('#status_log').css('height', log_height+'px');

    // Check the current size of the direction buttons
    var button_size =  $('#button_dir_0').width();


    // Calculate the available width and height of the div
    var div_width = $('#div_control_buttons').width();
    var div_height = $('#div_control_buttons').height();

    // Calculate the central position for the button pad
    var center_x = div_width * 0.5;
    var center_y = div_height * 0.4;

    // Now we reposition all the buttons in a circle
    for (var i=0; i<4; i++)
    {
        var angle = 90*i*2*Math.PI/360;
        var radius = button_size*1.5;
        var left_offset = (center_x - button_size/2) + radius * Math.cos(angle);
        var top_offset = (center_y  - button_size/2) + radius * Math.sin(angle);

        //console.log('left_offset:'+left_offset+'   top_offset:'+top_offset);

        $('#button_dir_'+i).css({left:left_offset, top: top_offset, transform:'rotate('+(90*i+90)+'deg)'});
    }

    button_size =  $('#button_stop').width();
    $('#button_stop').css({left:center_x-button_size/2, top: center_y-button_size/2});


    $('#canvas_heading').css({height: $('#table_heading').outerHeight()} );

    $('#canvas_sonars').css({height: $('#canvas_sonars').width() * 0.6} );

    // Calculate the y position for the increase/decrease buttons
    button_size =  $('#button_increase_linear').width();
    var control_y = div_height * 0.85 - button_size/2;


    // Linear speed buttons go on the left...
    $('#img_linear_control').css({left:div_width*0.20-button_size*0.5, top: div_height * 0.69});

    $('#button_decrease_linear').css({left:div_width*0.20-button_size*1.1, top: control_y});
    $('#button_increase_linear').css({left:div_width*0.20+button_size*0.1, top: control_y});

    // ... and radial speed buttons go on the rignt
    $('#img_radial_control').css({left:div_width*0.80-button_size*0.5, top: div_height * 0.69});
    $('#button_decrease_radial').css({left:div_width*0.80-button_size*1.1, top: control_y});
    $('#button_increase_radial').css({left:div_width*0.80+button_size*0.1, top: control_y});

}

function pressButton(event)
{
    var id = event.target.id;
    //console.log('pressButton:'+id);

    switch (id)
    {
        case 'button_dir_0': publishStringCommand(topic_teleop, 'RIGHT'); break;
        case 'button_dir_1': publishStringCommand(topic_teleop, 'DOWN'); break;
        case 'button_dir_2': publishStringCommand(topic_teleop, 'LEFT'); break;
        case 'button_dir_3': publishStringCommand(topic_teleop, 'UP'); break;
        case 'button_stop': publishStringCommand(topic_teleop, 'STOP'); break;

        case 'button_increase_linear': publishStringCommand(topic_teleop, 'VEL_UP'); break;
        case 'button_decrease_linear': publishStringCommand(topic_teleop, 'VEL_DOWN'); break;
        case 'button_increase_radial': publishStringCommand(topic_teleop, 'ANG_UP'); break;
        case 'button_decrease_radial': publishStringCommand(topic_teleop, 'ANG_DOWN'); break;

        //case 'button_disconnect': disconnectFromServer(); break;
    }
}

function roundRect(context, x, y, w, h, r) {
  if (w < 2 * r) r = w / 2;
  if (h < 2 * r) r = h / 2;
  context.beginPath();
  context.moveTo(x, y);
  context.arcTo(x+w, y,   x+w, y+h, r);
  context.arcTo(x+w, y+h, x,   y+h, r);
  context.arcTo(x,   y+h, x,   y,   r);
//  context.arcTo(x,   y,   x+w, y,   r);
  context.closePath();
}

// Fills a linear meter (canvas) up to a certain value
// The value must be a normalized number [0, 1]
function fillLinearMeter(id, value)
{
    // Get the canvas and the matching context
    var canvas = $('#'+id)[0];
    var context = canvas.getContext("2d");

    if (value > 1.0) value = 1.0;
    // Make the dimension of the canvas area equal to the css dimension
    var width = $('#'+id).width();
    var height = $('#'+id).height();
    $('#'+id).attr('width', width);
    $('#'+id).attr('height', height);

    // Clear the canvas area
    context.clearRect(0, 0, width, height);

    // If the value is positive, adjust it to the canvas dimensions
    if (value>0)
    {
        context.fillStyle = "#888888";
  //      context.fillRect(3, 3, (width-6)*value, height-6);

        roundRect(context, 0, 0, width*value, height, height/8);

        context.fill();
    }
}

// Draws the data from all the sonars in a single canvas
function drawSonars( id, value_array)
{
    //console.log(value_array[i]);
    // Get the canvas and the matching context
    var canvas = $('#'+id)[0];
    var context = canvas.getContext("2d");
    // Make the dimension of the canvas area equal to the css dimension
    var width = $('#'+id).width();
    var height = $('#'+id).height();

    // Common starting point for all the beams
    var origin_x = width * 0.5;
    var origin_y = height * 0.9;

    // Beam full length
    var beam_length = width * 0.45;


    $('#'+id).attr('width', width);
    $('#'+id).attr('height', height);

    // Clear the canvas area
    context.clearRect(0, 0, width, height);

    for (var i=0; i<5; i++)
    {

        // Values tipically in the range 15-80
        var value = Math.min(value_array[i], MAX_SONAR_VALUE) / MAX_SONAR_VALUE;

        //if (value_array[i] > 1.0) value = 1.0;
        context.lineWidth = 1;
        context.fillStyle="#888888";

        context.beginPath();
        context.moveTo( origin_x, origin_y );

        // Convert the angle to radians
        var angle = -140 + 25*i - 8;

        var alpha = angle*Math.PI/180;

        context.lineTo( origin_x+(beam_length*value*Math.cos(alpha)),
                        origin_y+(beam_length*value*Math.sin(alpha)) );

        angle = -140 + 25*i + 8;
        alpha = angle*Math.PI/180;
        context.lineTo( origin_x+(beam_length*value*Math.cos(alpha)),
                        origin_y+(beam_length*value*Math.sin(alpha)) );

        context.lineTo( origin_x, origin_y );

        context.fill();

    }
}

// Draws a pointer on a radial meter (canvas)
// The value can be any angle (in degrees)
function drawOrientation( id, angle)
{
    //console.log("drawOrientation:"+angle);
    // Get the canvas and the matching context
    var canvas = $('#'+id)[0];
    var context = canvas.getContext("2d");

    // Make the dimension of the canvas area equal to the css dimension
    var width = $('#'+id).width();
    var height = $('#'+id).height();

    $('#'+id).attr('width', width)           .attr('height', height);

    // Pointer full length
    var pointer_length = width * 0.45;

    // Clear the canvas area
    context.clearRect(0, 0, width, height);


    // Calculate the center of the canvas
    var center_x = width/2;
    var center_y = height/2;

    context.beginPath();
    context.arc(center_x, center_y, 8, 0, 2 * Math.PI, false);
    context.fillStyle = '#FFFFFF';
    context.fill();


    for (var i=0; i<19; i++)
    {
        var alpha = 30*i*Math.PI/180;
        context.beginPath();
        context.arc(center_x + pointer_length*Math.cos(alpha),
                    center_y + pointer_length*Math.sin(alpha), 3, 0, 2 * Math.PI, false);
        context.fillStyle = '#FFFFFF';
        context.fill();
    }

    var pointer_length = width * 0.35;

    // Convert the angle to radians
    var alpha = angle*Math.PI/180;

    context.lineWidth = 6;
    context.strokeStyle="#FFFFFF";
    context.lineCap="round";

    context.beginPath();
    context.moveTo(center_x, center_y);
    context.lineTo( center_x+(pointer_length*Math.cos(alpha)),
                    center_y+(pointer_length*Math.sin(alpha)) );
    context.stroke();
}
