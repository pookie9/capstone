//Note, the drawing was adapated from: http://www.williammalone.com/articles/create-html5-canvas-javascript-drawing-app/#demo-complete
//The background image upload taken from: https://stackoverflow.com/questions/22255580/javascript-upload-image-file-and-draw-it-into-a-canvas
var curSize = "normal";
var curHeight=5;
var radius=10;
var radii=new Array();
var clickX = new Array();
var clickY = new Array();
var heights=new Array();
var colorGreen = "#00ff00";
var colorYellow = "#ffcf33";
var colorBrown = "#986928";
var colorOrange="#ffa500";
var colorRed="#ff0000"
var curColor = colorGreen;

window.addEvent('load', function() {
var imageLoader = document.getElementById('imageLoader');
    imageLoader.addEventListener('change', handleImage, false);
var c = document.getElementById('canvas');
var ctx = c.getContext('2d');


function handleImage(e){
    var reader = new FileReader();
    reader.onload = function(event){
        var img = new Image();
        img.onload = function(){
            c.width = img.width;
            c.height = img.height;
            ctx.drawImage(img,0,0);
        }
        img.src = event.target.result;
    }
    reader.readAsDataURL(e.target.files[0]);     
}


context=ctx;
var clickSize = new Array();

var clickColor = new Array();


var clickDrag = new Array();
var paint;


$('#canvas').mouseleave(function(e){
  paint = false;
});

$('#canvas').mouseup(function(e){
  paint = false;
});
$('#canvas').mousemove(function(e){
  if(paint){
    addClick(e.pageX - this.offsetLeft, e.pageY - this.offsetTop, true);
    redraw();
  }
});
$('#canvas').mousedown(function(e){
  var mouseX = e.pageX - this.offsetLeft;
  var mouseY = e.pageY - this.offsetTop;
    
  paint = true;
  addClick(e.pageX - this.offsetLeft, e.pageY - this.offsetTop);
  redraw();
});
function redraw(){
  context.lineJoin = "round";
  /* context.lineWidth = 5; */
  for(var i=0; i < clickX.length; i++)
    {
    context.beginPath();
    if(clickDrag[i] && i){
      context.moveTo(clickX[i-1], clickY[i-1]);
    }else{
      context.moveTo(clickX[i]-1, clickY[i]);
    }
    context.lineTo(clickX[i], clickY[i]);
    context.closePath();
    context.strokeStyle = clickColor[i];
    context.lineWidth = radius;
    context.stroke();
  }
}
function addClick(x, y, dragging)
{
  clickX.push(x);
  clickY.push(y);
  heights.push(curHeight);
  clickDrag.push(dragging);
  clickColor.push(curColor);
  clickSize.push(curSize);
  radii.push(radius);
}

});
function small(){
    curSize="small";
    radius=5;
}
function huge(){
    curSize="huge";
    radius=15;
}

function normal(){
    curSize="normal";
    radius=10;
}
function green(){
    curColor=colorGreen;
    curHeight=5;
}
function yellow(){
    curColor=colorYellow;
    curHeight=10;
}
function orange(){
    curColor=colorOrange;
    curHeight=20;
}
function red(){
    curColor=colorRed;
    curHeight=50;
}
//outputs the heights in the correct format
function output(){
    out=new Array();
    for (var i=0; i<document.getElementById("canvas").width; i++){
	out.push(new Array());
	for (var j=0; j<document.getElementById("canvas").height; j++){
	    out[i].push(0);
	}
    }
    for(var i=0; i<clickX.length; i++){
	centerX=clickX[i];
	centerY=clickY[i];
	h=heights[i];
	r=radii[i];
	for(x=0; x<=r; x++){
	    for (y=0; y*y<=r*r-x*x; y++){
		out[centerX+x][centerY+y]=h; //have to do for each quadrant
		out[centerX-x][centerY+y]=h;
		out[centerX+x][centerY-y]=h;
		out[centerX-x][centerY-y]=h;
	    }
	}
    }
    height=document.getElementById("canvas").height;
    width=document.getElementById("canvas").width;
    document.write("<br/>");

    for (var i=0; i<width; i++){
	document.write(out[i]);
	document.write("<br/>");
    }

}
