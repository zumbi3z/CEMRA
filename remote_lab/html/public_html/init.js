window.onresize = function init()
{
	resizeInterface();
}



window.onload = function init()
{
	initialize_images();
	drawInterface();
	window.onresize();


	// Try to connect to the ROS server
	connectToServer();

	// Draws the sonars canvas (with default values)
    drawSonars('canvas_sonars', [1024, 1024, 1024, 1024, 1024]);

	// Draws the orientation canvas (with default values)
    drawOrientation('canvas_heading', 48);

}
