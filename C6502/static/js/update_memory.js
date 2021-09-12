var socket = io();

$(document).keypress((e) => {
	socket.emit("keypress", e.key);
});

$(document).ready(function(){
	$("#mem-read").on("submit", function(event) {
		event.preventDefault()
		console.log($(this).serialize())
		socket.emit("mem-addr-read", $(this).serialize())
	})

	socket.emit("mem-addr-read",$("#mem-read").serialize());
})




function Hex(d, padding, show=true) {
	var hex = Number(d).toString(16);
	hex = hex.padStart(padding, "0");
	if (show){
		return "$" + hex.toUpperCase();
	}else{
		return hex.toUpperCase();
	}
  }

function decPad(d, padding){
	var dec = Number(d).toString()
	text = "[" + dec.padStart(padding, " ") + "]";
	return text
}


socket.on("mem-addr-read", (value) => {
	var text = Hex(value) + " " + decPad(value, 3)
	$("#mem-value").text(text)
})

socket.on("reset", () => {
	console.log("RESET_SIG")
	$("#trace").html(null)
	$('#recent-memory').html(null)
	$("#stack-trace").html(null)
})

socket.on('update_info', (data) =>{
	const { 
		mem_block,
		flags: {N,V,B,D,I,Z,C},
		registers : {
			p_counter, 
			a_reg, 
			x_reg, 
			y_reg, 
			s_pointer },
		programs,
		trace
	 } = data; 
		
	
	$("#mem-cont").html(mem_block);
	
	(N === 1) ? $("span#flag_n").attr("class", "set") :$("span#flag_n").attr("class", "not-set");
	(V === 1) ? $("span#flag_v").attr("class", "set") :$("span#flag_v").attr("class", "not-set");
	(B === 1) ? $("span#flag_b").attr("class", "set") :$("span#flag_b").attr("class", "not-set");
	(D === 1) ? $("span#flag_d").attr("class", "set") :$("span#flag_d").attr("class", "not-set");
	(I === 1) ? $("span#flag_i").attr("class", "set") :$("span#flag_i").attr("class", "not-set");
	(Z === 1) ? $("span#flag_z").attr("class", "set") :$("span#flag_z").attr("class", "not-set");
	(C === 1) ? $("span#flag_c").attr("class", "set") :$("span#flag_c").attr("class", "not-set");
	$("#p_counter").text(Hex(p_counter, 4,true) + " " + decPad(p_counter, 5));
	$("#a_reg").text(Hex(a_reg, 2,true) + "   " + decPad(a_reg, 5));
	$("#x_reg").text(Hex(x_reg, 2,true) + "   " + decPad(x_reg, 5));
	$("#y_reg").text(Hex(y_reg, 2 ,true) + "   " + decPad(y_reg, 5));
	$("#s_pointer").text(Hex(s_pointer, 2,true) + "   " + decPad(s_pointer, 5));

	for (var i=0; i<16; i++){
		$("#prog-" + i.toString()).text(programs[i]);
	};


	if (trace){
		// Trace stack
		const registers = trace[1]
		const flags = trace[2]
		const clock = trace[3]
		const last_cycle = trace[4]
		const mem_access = trace[5]
		const stack = trace[6]
		const n_operations = trace[7]
		
		var trace_text = trace[0].padEnd(30, " ") + " " + n_operations
		trace_text += "|PC:" + Hex(registers[0],4) + "|SP:" + Hex(registers[1],2) 
		+ "|A:" + Hex(registers[2],2) + "|X:" + Hex(registers[3],2) + "|Y:" + Hex(registers[4],2) + "|"
		
		const f = ["N", "Z", "C", "I", "D", "B", "V"]
		for (let i = 0; i < flags.length; i++) {
			const flag_val = flags[i];
			if (flag_val === 1){
				trace_text += f[i]
			}
		}

		if ($("#trace").children().length === 500){
			$("#trace").children()[499].remove();
		}
		$("#trace").prepend("<div>" + trace_text + "</div>");
		

		//Recent memory access write
		if (mem_access){
			mem_access.forEach(element => {
				const mem_text = Hex(element[0], 4) + " = " + Hex(element[1], 2) + " " + decPad(element[1], 3);
				$('#recent-memory').prepend("<div>" + mem_text + "</div>");
			})

			if ($("#recent-memory").children().length >= 10){
				for (var i=$("#recent-memory").children().length - 1; i>=9; i--){
					$("#recent-memory").children()[i].remove();
				}
			}
		}

		//Stack content
		$("#stack-trace").html(null)
		if (stack){
			stack.forEach(element => {
				$("#stack-trace").append("<div>" + Hex(element, 2) + " " + decPad(element, 3) + "</div>")
			})
		}
	};
})

