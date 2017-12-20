function expandDiv(div, time)
{
    var height = $('#'+div).height();
    //console.log('height:'+);

    $('#'+div).stop().
              css({height:0, display:'block'}).
              animate( {height:height}, time, function() { $('#'+div).css({height:'auto'}); } );
}

function collapseDiv(div, time)
{
    var height = $('#'+div).height();
    //console.log('height:'+);

    $('#'+div).stop().
              animate( {height:0}, time, function() { $('#'+div).css({height:height, display:'none'}); } );
}
