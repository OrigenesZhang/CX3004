document.addEventListener('DOMContentLoaded', function initialize(e) {

	// Getting elements of arena
	var canvas = document.getElementById('canvas');
	var context = canvas.getContext('2d');
	const ROWS = 20, COLUMNS = 15;
	var roboPath = [];


	// Initializing arena to be unexplored
	var map = [];
	for (var i = 0; i < ROWS; i++){
		map[i] = [];
		for (var j = 0; j < COLUMNS; j++){
			map[i][j] = 0;
		}
	}

	function getStyle(cell) {
	    switch(cell) {
	        case 0: return "#1a1e24"; // unexplored
	        case 1: return "#F3F3F3"; // explored
	        case 2: return "#f44336"; // obstacle
	        case 3: return "#30807d"; // start
	        case 4: return "#08ae69"; // goal
	        case 5: return "#354458"; // robot
	        case 6: return "#7acdc8"; // path
	        case 7: return "#673ab7"; // way-point
	        default: return "#1a1e24";
	    }
	}

	function draw(map, center=null, head=null, area=null, time=null) {

		if (area){
    		document.getElementById('area').innerHTML = area+' %';
    	}
    	if (time){
    		document.getElementById('timer').innerHTML = time+' s';
    	}

        context.save();
        context.strokeStyle = "#252a33";
        context.lineWidth = 3;

        // Filling explored and unexplored cells
        for (var i = 0; i < ROWS; i++){
			for (var j = 0; j < COLUMNS; j++){
				context.beginPath();
				context.fillStyle = getStyle(map[i][j]);
				// First parameter is the x coordinate of top left of rectangle
				// Second parameter is the y coordinate of top left of rectangle
				// Third parameter is the width of the rectangle
				// Fourth parameter is the height of the rectangle
				context.rect(30 * j, 30 * i, 30, 30);
				context.fill();
				context.stroke();
				context.closePath();
			}
		}

		// add path
		if (roboPath){
			for(var i=0; i<roboPath.length; i++){
				pt = roboPath[i];
				if (map[pt[0]][pt[1]] != 7){
					context.beginPath();
					context.fillStyle = getStyle(6);
					context.rect(30 * pt[1], 30 * pt[0], 30, 30);
					context.fill();
					context.stroke();
					context.closePath();
				}
			}
		}

		// marking robot
		if (center && head){
			context.beginPath();
			context.fillStyle = getStyle(5);
			// Draw circle to represent robot
			context.moveTo(30*center[1] + 55, 30*center[0] + 15);
			context.arc(30*center[1] + 15, 30*center[0] + 15, 40, 0, 2 * Math.PI, true);
			context.fill();
			context.stroke();
			context.closePath();
			context.beginPath();
			context.fillStyle = getStyle(6);
			context.moveTo(30*head[1] + 20, 30*head[0] + 20);
			// Draw circle to represent head of robot
			context.arc(30*head[1] + 15, 30*head[0] + 20, 5, 0, 2 * Math.PI, true);
			context.fill();
			context.closePath();
		}
        context.restore();
    }

    (function () {
    var old = console.log;
    var logger = document.getElementById('logs-content');
    console.log = function () {
      for (var i = 0; i < arguments.length; i++) {
        if (typeof arguments[i] == 'object') {
            logger.innerHTML += (JSON && JSON.stringify ? JSON.stringify(arguments[i], undefined, 2) : arguments[i]) + '<br />';
        } else {
            logger.innerHTML += '<p>' + arguments[i] + '</p>';
        }
        logger.innerHTML += '<small>'+JSON.stringify(new Date())+'</small><br/><br/><hr/>';
      }
    }
	})();

    document.getElementById('mapName').addEventListener('click', function(e){
		document.getElementById('mapname').style.display = 'block';
	});

	document.getElementById('loadMap').addEventListener('click', function(e){
		document.getElementById('mapname').style.display = 'none';
		var name = document.getElementById('map-name').value;
		var r = new XMLHttpRequest();
		r.open("GET", "/lm?name="+name);
		r.send();
	});

	document.getElementById('reset').addEventListener('click', function(e){
		var r = new XMLHttpRequest();
		roboPath = [];
		r.open("GET", "/reset");
		r.send();
	});

	document.getElementById('waypoint').addEventListener('click', function(e){
		document.getElementById('way-point').style.display = 'block';
	});

	document.getElementById('fsp').addEventListener('click', function(e){
		document.getElementById('way-point').style.display = 'none';
		var x = document.getElementById('way-x').value;
		var y = document.getElementById('way-y').value;
		var r = new XMLHttpRequest();
		r.open("GET", "/fsp?x="+x+"&y="+y);
		r.send();
	});

	document.getElementById('start').addEventListener('click', function(e){
		document.getElementById('explore').style.display = 'block';
	});

	document.getElementById('exp_start').addEventListener('click', function(e){
		document.getElementById('explore').style.display = 'none';
		var step = document.getElementById('time_step').value;
		step = Number(1/step)
		var limit = Number(document.getElementById('time').value);
		var coverage = Number(document.getElementById('percentage').value);
		var r = new XMLHttpRequest();
		r.open("GET", "/start?step="+step+"&limit="+limit+"&coverage="+coverage);
		r.send();
	});

	function wsConnect() {
		// Generate a random ID for query parameter ID
		this.ws = new WebSocket("ws://"+window.location.host+"/websocket?Id=" + Math.floor(Math.random() * 100));
	    this.ws.onopen = function() {
	        ws.send("Initializing connection");
	    };
	    this.ws.onmessage = function(evt){
	    	var data = JSON.parse(evt.data);
	    	if (data.hasOwnProperty('log')){
	    		var msg = data.log;
	    		console.log(msg); // console.log has previously been changed to print in logs-content div
	    		//log(msg)
	    	}
	    	else{
	    		var map = JSON.parse(data.map);
		    	var center = JSON.parse(data.center);
		    	var head = JSON.parse(data.head);
		    	var area = data.area;
		    	var time = data.time;
		    	var msg = data.msg;
		    	roboPath.push(center); // Does not consider the intermediate steps if new center is not directly beside the previous position
		    	draw(map, center, head, area, time);
	    	}
	    };
	    this.ws.onerror = function(evt){
	    	console.log('WebSocket Error: ' + error);
	    }
	    this.ws.onclose = function(evt){
	    	setTimeout(wsConnect, 1000);
	    	console.log(evt);
	    }
	}

	var timeStepSlider = document.getElementById("time_step");
	var timeStepOutput = document.getElementById("time_step_value");
	timeStepOutput.innerHTML = timeStepSlider.value;

	timeStepSlider.oninput = function() {
        timeStepOutput.innerHTML = this.value;
	}
	
	var timeLimitSlider = document.getElementById("time");
	var timeLimitOutput = document.getElementById("time_limit_value");
	timeLimitOutput.innerHTML = timeLimitSlider.value;

	timeLimitSlider.oninput = function() {
        timeLimitOutput.innerHTML = this.value;
	}
	
	var percentageSlider = document.getElementById("percentage");
	var percentageOutput = document.getElementById("percentage_value");
	percentageOutput.innerHTML = percentageSlider.value;

	percentageSlider.oninput = function() {
        percentageOutput.innerHTML = this.value;
    }

	draw(map);
	wsConnect();

});
