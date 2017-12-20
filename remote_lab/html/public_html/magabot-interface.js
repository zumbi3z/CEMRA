// Maximum value read by the sonar sensors (used to calibrate the scale to display
// the information on the Sonar view module)
var MAX_SONAR_VALUE = 80;

// Size of the magabot (in pixels)
// This value needs to be calculated to make it match the arena dimensions
var MAGABOT_SIZE = 200;

// Current heading for the magabot (in degrees)
// This variable affects the pointer in the Heading module and the
// magabot icon in the Minimap module
var magabot_nav_angle = 45;

// Time it takes for a div to expand/collapse
var DIV_HIDE_TIME = 500;


// List of topics to be presented on the topic selection list
var topic_list =
[
    {name:"Camera 1", id:"cam1", selected:false},
    {name:"Camera 2", id:"cam2", selected:false}
]


// Images used to create the minimap
var image_magabot = new Image();
var image_arena = new Image();
var image_laser = new Image();


// Draws the whole UI
function drawInterface()
{
    // Disables the dragging of all HTML elements
    window.ondragstart = function() { return false; }

    var html = "";
    for (var i=0; i<topic_list.length; i++)
    {
        html += "<label><input type='checkbox' id='checkbox_"+topic_list[i].id+"' "+(topic_list[i].selected?"checked":"")+"/>"+topic_list[i].name+"</label><br>";
    }

    html += "<input type='button' id='button_start_record' class='topic_button' value='Start Recording'></input>";
    html += "<input type='button' id='button_stop_record' class='topic_button' value='Stop Recording'></input>";

    $('#div_topics').html(html);


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


    // Create all the speed control buttons
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

    // Load the images necessary for the minimap, making sure it is drawn as soon as they arrive
    image_magabot.onload = function() { drawMap(0.5, 0.5); };
    image_magabot.src = "img/magabot.svg";

    image_arena.onload = function() { drawMap(0.5, 0.5); };
    image_arena.src = "img/arena.png";

    image_laser.onload = function() { drawLaserMap(0.5, 0.5); };
    image_laser.src = "img/laser_readings.png";

    // Set the callback functions for the different buttons
    // Direction buttons
    $('.button_dir').on('click', pressButton);
    // Speed adjust buttons
    $('.button_adjust').on('click', pressButton);

    // Show/hide div buttons
    $('.toggle_image').click( headerClicked );

    // Clear system log button
    $('#img_clear_log').click( clearLog );

    // Bag record buttons
    $('#button_start_record').click( startRecordingBag );
    $('#button_stop_record').click( stopRecordingBag );

    $('#button_start_code').click( startCodeExec );
    $('#button_stop_code').click( stopCodeExec );

    $("#button_file").change(fileChange);

    //setInterval(animateMinimap, 100);

}

// Debug function to test the math
function animateMinimap()
{
    magabot_nav_angle+=10;
    drawMap(0.9, 0.9);
}


// Logs a string of data to the console and the log area on the page
function logData(str, type)
{
    var line = "<span class='"+type+"'>"+str+"</span><br>";

    $('#status_log').append(line);
    
    console.log(line);
    $("#status_log").scrollTop($("#status_log")[0].scrollHeight);
}

function logDataROS(str, type)
{
    var line = "<span class='"+type+"'>"+str.data+"</span><br>";

    $('#status_log').append(line);
    console.log(line);
    $("#status_log").scrollTop($("#status_log")[0].scrollHeight);
}


// Clears all the text the log area
function clearLog()
{
    $("#status_log").html("");
}



// Callback for when the file in the file selector changes
function fileChange(event)
{
    alert("New file selected:"+event.target.files[0].name);
}


// Processes the clicks on header buttons
// When a button a header button is clicked, the matching div is shown/hidden
// The ids between the headers and divs must be consistent:
// ex: header_test_content -> div_test_content
function headerClicked(event)
{
    // Calculate the id of the div from the id of the header
    var id = event.target.id;
    var name = "div_" + id.replace("img_toggle_", "");

    // If the matching div is currently visible, hide it
    if ($('#'+name).is(":visible"))
    {
        $('#'+id).attr("src", "img/btn_expand.svg");
        collapseDiv(name, DIV_HIDE_TIME);
    }
    // ... otherwise it's not visible, so let's show it
    else
    {
        $('#'+id).attr("src", "img/btn_collapse.svg");
        expandDiv(name, DIV_HIDE_TIME);
    }
}


// Draws the minimap, with the amagabot and the arena on the background
// The coordinates are normalized variables [0..1] in relation to the arena dimensions
function drawMap(x, y)
{
    var ctx = $("#canvas_map")[0].getContext('2d');
    console.log(ctx);
    ctx.canvas.width = 500;
    ctx.canvas.height = 500;

    var xpos = x * ctx.canvas.width;
    var ypos = y * ctx.canvas.height;

    ctx.fillStyle="#EEEEEE";
    ctx.fillRect(0, 0, ctx.canvas.width,  ctx.canvas.height);

    //ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

    ctx.drawImage(image_arena, 0, 0, ctx.canvas.width,  ctx.canvas.height);

    ctx.save();
    ctx.translate(xpos, ypos);
    //ctx.translate(-MAGABOT_SIZE/2, -MAGABOT_SIZE/2);
    ctx.rotate(magabot_nav_angle * Math.PI/180);
    ctx.drawImage(image_magabot, -MAGABOT_SIZE/2, -MAGABOT_SIZE/2, MAGABOT_SIZE, MAGABOT_SIZE);
    //ctx.drawImage(image_magabot, 0, 0, MAGABOT_SIZE, MAGABOT_SIZE);
    ctx.restore();
    
    

    /*
    ctx.strokeStyle = '#FF0000';
    ctx.beginPath();
    ctx.moveTo(xpos, ypos-50);
    ctx.lineTo(xpos, ypos+50);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(xpos-50, ypos);
    ctx.lineTo(xpos+50, ypos);
    ctx.stroke();
    */
}


function drawLaserMap(x, y)
{
    var ctx2 = $("#canvas_laser")[0].getContext('2d');
    //console.log(ctx2);
    ctx2.canvas.width = 500;
    ctx2.canvas.height = 500;

    var xpos = x * ctx2.canvas.width;
    var ypos = y * ctx2.canvas.height;

    ctx2.fillStyle="#EEEEEE";
    ctx2.fillRect(0, 0, ctx2.canvas.width,  ctx2.canvas.height);

    //ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

    ctx2.drawImage(image_laser, 0, 0, ctx2.canvas.width,  ctx2.canvas.height);

    ctx2.save();
    ctx2.restore();
}


function drawLaserReadings(laser_readings)
{
    var ctx = $("#canvas_laser")[0].getContext('2d');
    width = ctx.canvas.width;
    height = ctx.canvas.height;

    h = width / 2.0 - width * 0.05;
    str = laser_readings.header.frame_id.split(" ");
    x_max = parseFloat(str[0]);
    y_max = parseFloat(str[1]);
    //console.log(str);
    //console.log(y_max);
    
    //ctx.fillRect(width/1.5, height/1.5, 3, 3);

    //ctx.clearRect(0, 0, width, height);
    drawLaserMap(0.5, 0.5);

    n = Math.max(x_max, y_max);
    for (i = 0; i < laser_readings.poses.length; i++) {
       px = laser_readings.poses[i].position.x * h / n + width / 2;
       py = laser_readings.poses[i].position.y * h / n + height / 2;
       ctx.fillStyle = "#000000"
       ctx.fillRect(px, py, 3, 3);
       //console.log(px);
       //console.log(py);
    }
}


// Callback for when the interface is resized/initialized
// Some elements sizes ans positions need to be resized/moved depending on the window size
function resizeInterface()
{
    // Resize the video div to a 4/3 ratio
    $('#div_video_1').css('height', $('#div_video_1').width()*0.5625);
    $('#div_video_2').css('height', $('#div_video_2').width()*0.8182);

    // Resize the buttons div to a 4/3 ratio
    $('#div_control_buttons').css('height', $('#div_control_buttons').width() * 0.7);

    // Make the log window use all the remaining space on its column
    var log_height = $('#div_log').height() - $('#status_log').offset().top - $('#status_log').offset().left;
    $('#status_log').css('height', log_height+'px');
    //console.log('log_height:'+log_height);

    // Check the current size of the direction buttons
    var button_size =  $('#button_dir_0').width();

    // Calculate the available width and height of the div
    var div_width = $('#div_control_buttons').width();
    var div_height = $('#div_control_buttons').height();

    // Calculate the central position for the button pad
    var center_x = div_width * 0.3;
    var center_y = div_height * 0.5;

    // Now we reposition all the buttons in a circle
    for (var i=0; i<4; i++)
    {
        var angle = 90*i*2*Math.PI/360;
        var radius = button_size*1.5;
        var left_offset = (center_x - button_size/2) + radius * Math.cos(angle);
        var top_offset = (center_y  - button_size/2) + radius * Math.sin(angle);
        $('#button_dir_'+i).css({left:left_offset, top: top_offset, transform:'rotate('+(90*i+90)+'deg)'});
    }

    button_size =  $('#button_stop').width();
    $('#button_stop').css({left:center_x-button_size/2, top: center_y-button_size/2});


    $('#canvas_heading').css({height: $('#table_heading').outerHeight()} );

    // Maintain a form factor of 10/6 on the sonar canvas
    $('#canvas_sonars').css({height: $('#canvas_sonars').width() * 0.6} );

    // Calculate the y position for the increase/decrease buttons
    button_size =  $('#button_increase_linear').width();

    var control_y = div_height * 0.00;
    // Linear speed buttons go on the left...
    $('#img_linear_control').css({left:div_width*0.80-button_size*0.5, top: control_y});
    $('#button_decrease_linear').css({left:div_width*0.80-button_size*1.1, top: control_y + button_size*1.1});
    $('#button_increase_linear').css({left:div_width*0.80+button_size*0.1, top: control_y + button_size*1.1});

    control_y = div_height * 0.50;
    // ... and radial speed buttons go on the rignt
    $('#img_radial_control').css({left:div_width*0.80-button_size*0.5, top: control_y});
    $('#button_decrease_radial').css({left:div_width*0.80-button_size*1.1, top: control_y + button_size*1.1});
    $('#button_increase_radial').css({left:div_width*0.80+button_size*0.1, top: control_y + button_size*1.1});
}


// Callback for when a teleop button is pressed
function pressButton(event)
{
    var id = event.target.id;
    console.log('pressButton:'+id);

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
    }
}


// Draws a rectangle with round corners (configurable radius) in a specfied context
function roundRect(context, x, y, w, h, r)
{
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
function fillLinearMeter( id, value)
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

        // Each slice starts on the same center point
        context.beginPath();
        context.moveTo( origin_x, origin_y );

        // Adjust the angle to the position we want and convert to radians
        var angle = -140 + 25*i - 8;
        var alpha = angle*Math.PI/180;

        // Second edge of the triangle
        context.lineTo( origin_x+(beam_length*value*Math.cos(alpha)),
                        origin_y+(beam_length*value*Math.sin(alpha)) );

        // Adjust the angle to the position we want and convert to radians
        angle = -140 + 25*i + 8;
        alpha = angle*Math.PI/180;

        // Third edge of the triangle
        context.lineTo( origin_x+(beam_length*value*Math.cos(alpha)),
                        origin_y+(beam_length*value*Math.sin(alpha)) );

        // Close the triangle by going back to the origin
        context.lineTo( origin_x, origin_y );

        context.fill();
    }
}


// Draws a pointer on a radial meter (canvas)
// The value can be any angle (in degrees)
function drawOrientation( id, angle)
{
    console.log("drawOrientation:"+angle);

    // The angle to display on the minimap is the same as this one
    magabot_nav_angle = angle;

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

    // Draw all the dots that serve as reference points
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

    // Finally, we draw the pointer in the desired angle
    context.beginPath();
    context.moveTo(center_x, center_y);
    context.lineTo( center_x+(pointer_length*Math.cos(alpha)),
                    center_y+(pointer_length*Math.sin(alpha)) );
    context.stroke();
}

function initialize_images(){
        img = new Image();
        img2 = new Image();
        img.src = "http://sscpc0.ist.utl.pt:28001/stream?topic=camera1/image_raw";
        img2.src = "http://sscpc0.ist.utl.pt:28001/stream?topic=camera2/image_raw";
}

function refreshCanvas(){
    //objDiv1 = document.getElementById("status_log");
    //objDiv1.scrollTop = objDiv1.scrollHeight;
    //objDiv2 = document.getElementById("div_feedback");
    //objDiv2.scrollTop = objDiv2.scrollHeight;
    ctx = document.getElementById('test_canvas').getContext('2d');
    ctx.drawImage(img, -40, 0, 400, 150);
}

function refreshCanvas2(){
        ctx2 = document.getElementById('test_canvas2').getContext('2d');
    ctx2.drawImage(img2, 0, 0, 300, 150);
}

window.setInterval("refreshCanvas()", 1);
window.setInterval("refreshCanvas2()", 1);
