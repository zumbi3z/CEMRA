window.onresize = function resize() 
{
	resizeInterface();
}

window.onload = function init() 
{
	initialize_images();
	drawInterface();
	window.onresize();

	$('#button_connect').click( connectToServer );

	drawSonars('canvas_sonars', [1024, 1024, 1024, 1024, 1024]);

	drawOrientation('canvas_heading', 48);

}
