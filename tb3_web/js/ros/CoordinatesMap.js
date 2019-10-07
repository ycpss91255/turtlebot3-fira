//get coordinates_Map data
var map = document.getElementById("coordinates_Map");
//get player data
var view = document.getElementById("player");

var Camera = document.getElementById("View-or-Map");
function Map_control(){
    if(view.src.slice(-14) == "img/ground.png"){
        console.log("Map Sending");
    }
    else{
        console.log("Don't touch me");
    }
}
var num = 0;
map.addEventListener("mousedown",function(e){
    if(view.src.slice(-14) == "img/ground.png"){
        let ctx = map.getContext("2d");

        let x0 = 320;
        let y0 = 448;
		
        mouse_x = e.offsetX;
        mouse_y = e.offsetY;
		
		let coord_x = parseInt((mouse_x-x0)/1.28);
		let coord_y = parseInt((mouse_y-y0)/1.28);
		
        ctx.beginPath();
        ctx.lineWidth = 2;
        ctx.strokeStyle ="rgb(55, 0, 255)";
		ctx.arc(mouse_x, mouse_y, 15, 0, 2*Math.PI);
		//ctx.moveTo(mouse_x, mouse_y);
		//ctx.lineTo(x0, y0);
		ctx.font="25px Georgia";
		ctx.textAlign = "center"
        ctx.fillStyle="black";
		ctx.fillText(num.toString(), mouse_x, mouse_y+25/4);
        ctx.stroke();
		ctx.closePath();
		num++;
		
        console.log(e);
        console.log(e.offsetX,e.offsetY);
		console.log(coord_x,coord_y); 
    }
})
