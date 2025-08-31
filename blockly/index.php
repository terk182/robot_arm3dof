<?php
error_reporting(0);
 $arm_ip = $_GET['ip'] ;
if(is_null($arm_ip)){
    $arm_ip = "no_arm";
}
?>

<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <script src="blockly/blockly_compressed.js"></script>
    <script src="blockly/blocks_compressed.js"></script>
    <script src="blockly/javascript_compressed.js"></script>
    <script src="blockly/msg/en.js"></script>
    <style>
        /* กำหนดสไตล์สำหรับปุ่ม */
        .control-button {
            background-color: #4CAF50; /* สีพื้นหลัง */
            border: none; /* ไม่มีขอบ */
            color: white; /* สีข้อความ */
            padding: 15px 30px; /* ระยะห่างภายในปุ่ม */
            text-align: center; /* จัดวางข้อความให้อยู่ตรงกลาง */
            text-decoration: none; /* ไม่มีเส้นใต้ */
            display: inline-block; /* จัดให้อยู่ในแนวเดียวกัน */
            font-size: 16px; /* ขนาดตัวอักษร */
            margin: 10px 5px; /* ระยะห่างระหว่างปุ่ม */
            cursor: pointer; /* เปลี่ยนเคอร์เซอร์เมื่อชี้ที่ปุ่ม */
            border-radius: 12px; /* มุมโค้งมน */
            transition: background-color 0.3s, transform 0.3s; /* การเปลี่ยนสีและการเคลื่อนไหว */
        }

        /* สไตล์เมื่อมีการชี้ที่ปุ่ม */
        .control-button:hover {
            background-color: #45a049; /* สีเมื่อชี้ */
            transform: scale(1.1); /* ขยายปุ่มเมื่อชี้ */
        }

        /* สไตล์เมื่อกดปุ่ม */
        .control-button:active {
            background-color: #3e8e41; /* สีเมื่อกด */
            transform: scale(0.9); /* ย่อปุ่มเมื่อกด */
        }
    </style>
</head>
<body>
    <h1>Blockly Robotic Arm Control <?php echo $arm_ip ?></h1>

    
    <button class="control-button" onclick="executeCode()">Run Code</button>
    <button class="control-button" onclick="saveWorkspace()">Save</button>
    <button class="control-button" onclick="loadWorkspace()">Load</button>

    <input type="file" id="fileInput" style="display: none;" onchange="loadFile(event)">
    <div id="blocklyDiv" style="height: 540px; width: 1000px;"></div>
    <xml id="toolbox" style="display: none">
        <category name="การควบคุม" colour="200">
            <block type="move_arm"></block>
            <block type="gripper_value_control"></block>
            <block type="gripper_on_off_control"></block>
            <block type="home_position"></block>
            <block type="set_zero"></block>
            <block type="joint_angles"></block>
            <block type="inverse_kinematics"></block>
            <block type="input_block_number"></block>
            <block type="output_block"></block>
            <block type="delay_block"></block>
            
        </category>
        <category name="เงือนไข" colour="120">
            
            <block type="controls_for"></block>
            <block type="controls_if_high_low"></block>
            <block type="controls_if_input_high_low"></block>
           
            <block type="controls_repeat_ext"></block>
            <block type="controls_repeat_boolean"></block>
            <block type="controls_while"></block>
       
        </category>
        <category name="ตัวแปร" colour="360">
            <block type="int_value"></block>
        </category>
    </xml>


    <p id="arm_ip"><?php echo $arm_ip ?></p>
    <script>
        var ARM_MODE ="JOINT"
        const esp32IP = document.getElementById('arm_ip').innerHTML;
        var workspace = Blockly.inject('blocklyDiv', {
            toolbox: document.getElementById('toolbox')
        });
   // ฟังก์ชัน Save Workspace
   function saveWorkspace() {
            var xml = Blockly.Xml.workspaceToDom(workspace);
            var xmlText = Blockly.Xml.domToPrettyText(xml);
            
            var blob = new Blob([xmlText], {type: 'text/xml'});
            var link = document.createElement('a');
            link.href = URL.createObjectURL(blob);
            link.download = 'blockly_workspace.xml';
            link.click();
        }

        // ฟังก์ชัน Load Workspace
        function loadWorkspace() {
            document.getElementById('fileInput').click();
        }

        function loadFile(event) {
            var file = event.target.files[0];
    var reader = new FileReader();
    reader.onload = function(event) {
        var xmlText = event.target.result;
        var xml = Blockly.utils.xml.textToDom(xmlText); // แก้ไขบรรทัดนี้
        Blockly.Xml.domToWorkspace(xml, workspace);
    };
    reader.readAsText(file);
        }


        Blockly.defineBlocksWithJsonArray([
            {
                "type": "move_arm",
                "message0": "Move arm using %1 %2",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "MODE",
                        "options": [
                            ["Joint Angles", "JOINT"],
                            ["Inverse Kinematics", "IK"]
                        ]
                    },
                    {
                        "type": "input_value",
                        "name": "VALUES"
                    }
                ],
                "inputsInline": true,
                "previousStatement": null,
                "nextStatement": null,
                "colour": 120,
                "tooltip": "Move robotic arm with joint angles or inverse kinematics",
                "helpUrl": ""
            }
        ]);
        // บล็อกสำหรับการควบคุม Gripper แบบ Value
        Blockly.defineBlocksWithJsonArray([
            {
                "type": "gripper_value_control",
                "message0": "Set gripper to %1",
                "args0": [
                    {"type": "field_number", "name": "GRIPPER", "value": 0, "min": 0, "max": 180}
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": 60,
                "tooltip": "Control the gripper with a specific value",
                "helpUrl": ""
            }
        ]);

        // บล็อกสำหรับการควบคุม Gripper แบบ On/Off
        Blockly.defineBlocksWithJsonArray([
            {
                "type": "gripper_on_off_control",
                "message0": "Set gripper %1",
                "args0": [
                    {
                        "type": "field_dropdown", 
                        "name": "GRIPPER_STATE", 
                        "options": [
                            ["Open", "OPEN"],
                            ["Close", "CLOSE"]
                        ]
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": 60,
                "tooltip": "Control the gripper to open or close",
                "helpUrl": ""
            }
        ]);

        // บล็อกสำหรับการสร้าง delay
        Blockly.defineBlocksWithJsonArray([
            {
                "type": "delay_block",
                "message0": "Delay %1 ms",
                "args0": [
                    {"type": "field_number", "name": "DELAY", "value": 1000}
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": 210,
                "tooltip": "Add delay in milliseconds",
                "helpUrl": ""
            }
        ]);

        // บล็อกสำหรับการตั้งค่า Home Position
        Blockly.defineBlocksWithJsonArray([
            {
                "type": "home_position",
                "message0": "Move to Home Position",
                "previousStatement": null,
                "nextStatement": null,
                "colour": 300,
                "tooltip": "Move the robotic arm to the home position",
                "helpUrl": ""
            }
        ]);
        // บล็อกสำหรับการตั้งค่า Zero Position
        Blockly.defineBlocksWithJsonArray([
            {
                "type": "set_zero",
                "message0": "Set to Zero Position",
                "previousStatement": null,
                "nextStatement": null,
                "colour": 330,
                "tooltip": "Set the robotic arm to zero position",
                "helpUrl": ""
            }
        ]);
        // บล็อกสำหรับการทำซ้ำ (For loop)
        Blockly.defineBlocksWithJsonArray([{
            "type": "controls_for",
            "message0": "for %1 from %2 to %3 by %4",
            "args0": [
                {
                    "type": "field_variable",
                    "name": "VAR",
                    "variable": "i"
                },
                {
                    "type": "input_value",
                    "name": "FROM",
                    "check": "Number"
                },
                {
                    "type": "input_value",
                    "name": "TO",
                    "check": "Number"
                },
                {
                    "type": "input_value",
                    "name": "BY",
                    "check": "Number"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 120,
            "tooltip": "Repeats from a start number to an end number by a specified step.",
            "helpUrl": "",
            "inputsInline": true,
            "extensions": ["contextMenu_newGetVariableBlock"]
        }]);
        // บล็อกสำหรับการทำซ้ำ (While loop)
        Blockly.defineBlocksWithJsonArray([{
            "type": "controls_whileUntil",
            "message0": "while %1",
            "args0": [
                {
                    "type": "input_value",
                    "name": "CONDITION",
                    "check": "Boolean"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 210,
            "tooltip": "Repeats while a condition is true.",
            "helpUrl": ""
        }]);
        // สร้างบล็อกสำหรับ while loop
        Blockly.defineBlocksWithJsonArray([{
            "type": "controls_while",
            "message0": "while %1",
            "args0": [
                {
                    "type": "input_value",
                    "name": "CONDITION",
                    "check": "Boolean"
                }
            ],
            "message1": "do %1",
            "args1": [
                {
                    "type": "input_statement",
                    "name": "DO"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 210,
            "tooltip": "Repeat while the condition is true",
            "helpUrl": ""
        }]);

        // บล็อกสำหรับรับ Input
// บล็อกสำหรับรับ Input แบบ Number
        Blockly.defineBlocksWithJsonArray([{
            "type": "input_block_number",
            "message0": "Input number %1",
            "args0": [
                {
                    "type": "field_number",
                    "name": "INPUT_VALUE",
                    "value": 0
                }
            ],
            "output": "Number",
            "colour": 160,
            "tooltip": "Receive input value as a number",
            "helpUrl": ""
        }]);


// บล็อกสำหรับส่ง Output ที่สามารถเลือกรูปแบบเป็น int หรือ on/off ได้
        Blockly.defineBlocksWithJsonArray([{
            "type": "output_block",
            "message0": "Output %1 as %2",
            "args0": [
                {
                    "type": "input_value",
                    "name": "OUTPUT_VALUE"
                },
                {
                    "type": "field_dropdown",
                    "name": "STATE",
                    "options": [
                        ["HIGH", "HIGH"],
                        ["LOW", "LOW"]
                    ]
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 230,
            "tooltip": "Send output value as an integer or on/off",
            "helpUrl": ""
        }]);

        // สร้างบล็อก 'if'
        // สร้างบล็อก 'if' ที่ตรวจสอบค่าเป็น HIGH หรือ LOW
        Blockly.defineBlocksWithJsonArray([{
            "type": "controls_if_high_low",
            "message0": "if input %1 is %2 then",
            "args0": [
                {
                    "type": "input_value",
                    "name": "INPUT"
                },
                {
                    "type": "field_dropdown",
                    "name": "STATE",
                    "options": [
                        ["HIGH", "HIGH"],
                        ["LOW", "LOW"]
                    ]
                }
            ],
            "message1": "do %1",
            "args1": [
                {
                    "type": "input_statement",
                    "name": "DO"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 210,
            "tooltip": "If the input is HIGH or LOW, then do some statements",
            "helpUrl": ""
        }]);
        // สร้างบล็อกสำหรับ repeat loop
        Blockly.defineBlocksWithJsonArray([{
            "type": "controls_repeat_ext",
            "message0": "repeat %1 times",
            "args0": [
                {
                    "type": "input_value",
                    "name": "TIMES",
                    "check": "Number"
                }
            ],
            "message1": "do %1",
            "args1": [
                {
                    "type": "input_statement",
                    "name": "DO"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 120,
            "tooltip": "Repeat a set of statements a specific number of times",
            "helpUrl": ""
        }]);

        // สร้างบล็อกสำหรับ if-else ที่ตรวจสอบ input เป็น HIGH หรือ LOW
        Blockly.defineBlocksWithJsonArray([{
            "type": "controls_if_input_high_low",
            "message0": "if input %1 is %2 then",
            "args0": [
                {
                    "type": "input_value",
                    "name": "INPUT"
                },
                {
                    "type": "field_dropdown",
                    "name": "STATE",
                    "options": [
                        ["HIGH", "HIGH"],
                        ["LOW", "LOW"]
                    ]
                }
            ],
            "message1": "do %1",
            "args1": [
                {
                    "type": "input_statement",
                    "name": "DO"
                }
            ],
           
            "message2": "else %1",
            "args2": [
                {
                    "type": "input_statement",
                    "name": "ELSE"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 210,
            "tooltip": "If the input value equals HIGH or LOW, execute the corresponding set of statements",
            "helpUrl": ""
        }]);


   
// สร้างบล็อก Int
        Blockly.defineBlocksWithJsonArray([{
            "type": "int_value",
            "message0": "int %1",
            "args0": [
                {
                    "type": "field_number",
                    "name": "INT",
                    "value": 0
                }
            ],
            "output": "Number",
            "colour": 230,
            "tooltip": "An integer value",
            "helpUrl": ""
        }]);
        // Block for specifying joint angles
        Blockly.defineBlocksWithJsonArray([
            {
                "type": "joint_angles",
                "message0": "Joint 1 %1 Joint 2 %2 Joint 3 %3 speed %4 acceleration %5",
                "args0": [
                    {"type": "field_number", "name": "THETA1", "value": 0},
                    {"type": "field_number", "name": "THETA2", "value": 0},
                    {"type": "field_number", "name": "THETA3", "value": 0},
                    {"type": "field_number", "name": "SPEED", "value": 1000},
                    {"type": "field_number", "name": "ACCELERATION", "value": 500}
                ],
                "output": null,
                "colour": 240,
                "tooltip": "Specify joint angles for movement",
                "helpUrl": ""
            }
        ]);

        // Block for specifying inverse kinematics values
        Blockly.defineBlocksWithJsonArray([
            {
                "type": "inverse_kinematics",
                "message0": "X %1 Y %2 Z %3 speed %4 acceleration %5",
                "args0": [
                    {"type": "field_number", "name": "X", "value": 0},
                    {"type": "field_number", "name": "Y", "value": 0},
                    {"type": "field_number", "name": "Z", "value": 0},
                    {"type": "field_number", "name": "SPEED", "value": 1000},
                    {"type": "field_number", "name": "ACCELERATION", "value": 500}
                ],
                "output": null,
                "colour": 240,
                "tooltip": "Specify position for inverse kinematics",
                "helpUrl": ""
            }
        ]);
        // สร้างบล็อกสำหรับ repeat loop ที่ให้เลือก true หรือ false
        Blockly.defineBlocksWithJsonArray([{
            "type": "controls_repeat_boolean",
            "message0": "repeat while %1",
            "args0": [
                {
                    "type": "field_dropdown",
                    "name": "CONDITION",
                    "options": [
                        ["true", "TRUE"],
                        ["false", "FALSE"]
                    ]
                }
            ],
            "message1": "do %1",
            "args1": [
                {
                    "type": "input_statement",
                    "name": "DO"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 120,
            "tooltip": "Repeat a set of statements while the condition is true or false",
            "helpUrl": ""
        }]);
     // การแปลงบล็อก 'if' เป็นโค้ด JavaScript
     Blockly.JavaScript['controls_if_high_low'] = function(block) {
            var input = Blockly.JavaScript.valueToCode(block, 'INPUT', Blockly.JavaScript.ORDER_ATOMIC) || '0';
            var state = block.getFieldValue('STATE');
            
            // ตรวจสอบเงื่อนไขที่เลือก (HIGH หรือ LOW)
            var condition = (state === 'HIGH') ? `${input} === 1` : `${input} === 0`;
            
            var statements_do = Blockly.JavaScript.statementToCode(block, 'DO');
            var code = `if (${condition}) {\n${statements_do}}\n`;
            return code;
        };

        // การแปลงบล็อก repeat loop เป็นโค้ด JavaScript
        Blockly.JavaScript['controls_repeat_ext'] = function(block) {
            var repeats = Blockly.JavaScript.valueToCode(block, 'TIMES', Blockly.JavaScript.ORDER_ATOMIC) || '0';
            var branch = Blockly.JavaScript.statementToCode(block, 'DO');
            var code = `for (var i = 0; i < ${repeats}; i++) {\n${branch}}\n`;
            return code;
        };

        // การแปลงบล็อก Input เป็นโค้ด JavaScript
        Blockly.JavaScript['input_block_number'] = function(block) {
            var inputValue = block.getFieldValue('INPUT_VALUE');
            var code = `${inputValue}`;
            return [code, Blockly.JavaScript.ORDER_ATOMIC];
        };

        // การแปลงบล็อก Output เป็นโค้ด JavaScript
        Blockly.JavaScript['output_block'] = function(block) {
            var outputValue = Blockly.JavaScript.valueToCode(block, 'OUTPUT_VALUE', Blockly.JavaScript.ORDER_ATOMIC);
            var outputType = block.getFieldValue('OUTPUT_TYPE');

            // ตรวจสอบว่าผู้ใช้เลือก 'int' หรือ 'on/off'
            var code = '';
            if (outputType === 'INT') {
                code = `console.log(${outputValue});\n`; // สำหรับ output เป็นจำนวนเต็ม (int)
            } else if (outputType === 'ONOFF') {
                var onOffValue = (outputValue == 1) ? 'ON' : 'OFF'; // แปลงค่าเป็น ON หรือ OFF
                code = `console.log("${onOffValue}");\n`;
            }
            return code;
        };
        // การแปลงบล็อก 'while loop' เป็นโค้ด JavaScript
        Blockly.JavaScript['controls_whileUntil'] = function(block) {
            var condition = Blockly.JavaScript.valueToCode(block, 'CONDITION', Blockly.JavaScript.ORDER_ATOMIC) || 'false';
            var branch = Blockly.JavaScript.statementToCode(block, 'DO');
            
            var code = `while (${condition}) {\n${branch}}\n`;
            return code;
        };
        // การแปลงบล็อก while loop เป็นโค้ด JavaScript
        Blockly.JavaScript['controls_while'] = function(block) {
            var condition = Blockly.JavaScript.valueToCode(block, 'CONDITION', Blockly.JavaScript.ORDER_NONE) || 'false';
            var statements_do = Blockly.JavaScript.statementToCode(block, 'DO');
            var code = `while (${condition}) {\n${statements_do}}\n`;
            return code;
        };

        // การแปลงบล็อก 'for loop' เป็นโค้ด JavaScript
        Blockly.JavaScript['controls_for'] = function(block) {
            var variable = Blockly.JavaScript.variableDB_.getName(block.getFieldValue('VAR'), Blockly.VARIABLE_CATEGORY_NAME);
            var from = Blockly.JavaScript.valueToCode(block, 'FROM', Blockly.JavaScript.ORDER_ATOMIC) || '0';
            var to = Blockly.JavaScript.valueToCode(block, 'TO', Blockly.JavaScript.ORDER_ATOMIC) || '0';
            var by = Blockly.JavaScript.valueToCode(block, 'BY', Blockly.JavaScript.ORDER_ATOMIC) || '1';
            
            var branch = Blockly.JavaScript.statementToCode(block, 'DO');
            var code = `for (var ${variable} = ${from}; ${variable} <= ${to}; ${variable} += ${by}) {\n${branch}}\n`;
            return code;
        };
        // การแปลงบล็อก if input high/low do else เป็นโค้ด JavaScript
        Blockly.JavaScript['controls_if_input_high_low'] = function(block) {
            var input = Blockly.JavaScript.valueToCode(block, 'INPUT', Blockly.JavaScript.ORDER_ATOMIC) || '""';
            var compareValue = block.getFieldValue('COMPARE_VALUE');
            var statements_do = Blockly.JavaScript.statementToCode(block, 'DO');
            var statements_else = Blockly.JavaScript.statementToCode(block, 'ELSE');
            
            // แปลงค่า HIGH เป็น 1 และ LOW เป็น 0
            var compareValueNumber = (compareValue === 'HIGH') ? '1' : '0';
            var code = `if (${input} == ${compareValueNumber}) {\n${statements_do}} else {\n${statements_else}}\n`;
            return code;
        };


        Blockly.JavaScript['gripper_value_control'] = function(block) {
            var gripper = block.getFieldValue('GRIPPER');
            var code = `sendGripperValueCommand(${gripper});\n`;
            return code;
        };

        Blockly.JavaScript['gripper_on_off_control'] = function(block) {
            var gripperState = block.getFieldValue('GRIPPER_STATE');
            var gripperValue = gripperState === "OPEN" ? 0 : 180; // เปิดเป็น 0 และปิดเป็น 180
            var code = `sendGripperValueCommand(${gripperValue});\n`;
            return code;
        };

        Blockly.JavaScript['delay_block'] = function(block) {
            var delay = block.getFieldValue('DELAY');
            var code = `sendDelayCommand(${delay});\n`;
            return code;
        };

        Blockly.JavaScript['home_position'] = function(block) {
            var code = `sendHomePositionCommand();\n`;
            return code;
        };
        // การแปลงบล็อก 'move_arm' เป็นโค้ด JavaScript
        Blockly.JavaScript['move_arm'] = function(block) {
            var mode = block.getFieldValue('MODE');
            var values = Blockly.JavaScript.valueToCode(block, 'VALUES', Blockly.JavaScript.ORDER_ATOMIC);

            var code = '';
            if (mode === 'JOINT') {
                code = `sendMoveCommand(${values});\n`;
            } else if (mode === 'IK') {
                code = `sendMoveCommand(${values});\n`;
            }
            return code;
        };
        // การแปลงบล็อก repeat while true/false เป็นโค้ด JavaScript
        Blockly.JavaScript['controls_repeat_boolean'] = function(block) {
            var condition = block.getFieldValue('CONDITION') === 'TRUE';
            var statements_do = Blockly.JavaScript.statementToCode(block, 'DO');
            var code = `while (${condition}) {\n${statements_do}}\n`;
            return code;
        };

        Blockly.JavaScript['joint_angles'] = function(block) {
            var theta1 = block.getFieldValue('THETA1');
            var theta2 = block.getFieldValue('THETA2');
            var theta3 = block.getFieldValue('THETA3');
            var speed = block.getFieldValue('SPEED');
            var acceleration = block.getFieldValue('ACCELERATION');
            var code = `${theta1}, ${theta2}, ${theta3}, ${speed}, ${acceleration}`;
            return [code, Blockly.JavaScript.ORDER_NONE];
        };

        Blockly.JavaScript['inverse_kinematics'] = function(block) {
            var x = block.getFieldValue('X');
            var y = block.getFieldValue('Y');
            var z = block.getFieldValue('Z');
            var speed = block.getFieldValue('SPEED');
            var acceleration = block.getFieldValue('ACCELERATION');
            var code = `${x}, ${y}, ${z}, ${speed}, ${acceleration}`;
            return [code, Blockly.JavaScript.ORDER_NONE];
        };
        // การแปลงบล็อก Int เป็นโค้ด JavaScript
        Blockly.JavaScript['int_value'] = function(block) {
            var intValue = block.getFieldValue('INT');
            var code = `${intValue}`;
            return [code, Blockly.JavaScript.ORDER_ATOMIC];
        };
        function executeCode() {
          

            var xml1 = Blockly.Xml.workspaceToDom(workspace);
            var xmlText1 = Blockly.Xml.domToPrettyText(xml1);
            var blockObj = xmlToObj(xmlText1);
            console.log(blockObj);
            executeCommands(blockObj);
        }

        
   

        function sendDelayCommand(delay) {
            setTimeout(() => {
                console.log(`Delay of ${delay} ms completed`);
            }, delay);
        }

        function sendHomePositionCommand() {
            console.log("sendHomePositionCommand");
            console.log(esp32IP);
            fetch(`http://${esp32IP}/homePosition`)
                .then(response => {
                    if (response.ok) {
                        console.log("Moved to Home Position successfully!");
                    } else {
                        console.error("Failed to move to Home Position.");
                    }
                });
        }
        // ฟังก์ชันที่ใช้ในการแปลง XML เป็น Object
function xmlToObj(xmlText) {
    var parser = new DOMParser();
    var xmlDoc = parser.parseFromString(xmlText, "text/xml");

    var workspace = new Blockly.Workspace();
    Blockly.Xml.domToWorkspace(xmlDoc.documentElement, workspace);

    var blocks = workspace.getAllBlocks(false);
    var blockArray = blocks.map(block => blockToObj(block));

    return blockArray;
}

// ฟังก์ชันที่ใช้ในการแปลงแต่ละบล็อกเป็น Object
function blockToObj(block) {
    var obj = {
        type: block.type,
        id: block.id,
        fields: {},
        inputs: {},
        next: null
    };

    // ดึงข้อมูลฟิลด์ของบล็อก
    block.inputList.forEach(input => {
        input.fieldRow.forEach(field => {
            obj.fields[field.name] = field.getValue();
        });
    });

    // ดึงข้อมูลอินพุตของบล็อก
    block.inputList.forEach(input => {
        if (input.connection && input.connection.targetBlock()) {
            obj.inputs[input.name] = blockToObj(input.connection.targetBlock());
        }
    });

    // ดึงข้อมูลบล็อกถัดไป
    if (block.getNextBlock()) {
        obj.next = blockToObj(block.getNextBlock());
    }

    return obj;
}

// ฟังก์ชันสำหรับส่งคำสั่งการเคลื่อนที่ไปยัง ESP32
function sendMoveCommand(theta1, theta2, theta3, speed, acceleration,mode) {
    console.log("sendMoveCommand");
    console.log(esp32IP);
    let txt = `http://${esp32IP}/move?theta1=${theta1}&theta2=${theta2}&theta3=${theta3}&speed=${speed}&acceleration=${acceleration}&mode=${mode}`;
    console.log(txt);


    // fetch(txt, {
    //         method: 'GET',
    //         mode: 'no-cors'
    //     }).then(response => {
    //         console.log("Command sent successfully!");
    //     }).catch(error => {
    //         console.error("Failed to send command:", error);
    //     });

   
}
function sendForCommand(theta1, theta2, theta3) {
  
    console.log(esp32IP);
    let txt = `http://${esp32IP}/for?theta1=${theta1}&theta2=${theta2}&theta3=${theta3}`;
    console.log(txt);


    fetch(txt, {
            method: 'GET',
            mode: 'no-cors'
        }).then(response => {
            console.log("Command sent successfully!");
        }).catch(error => {
            console.error("Failed to send command:", error);
        });

   
}
// ฟังก์ชันสำหรับส่งคำสั่งการควบคุม Gripper ไปยัง ESP32
function sendGripperValueCommand(gripperValue) {

        let  url = `http://${esp32IP}/controlGripper?gripper=${gripperValue}`
        fetch(url, {
            method: 'GET',
            mode: 'no-cors'
        }).then(response => {
            console.log("Command sent successfully!");
        }).catch(error => {
            console.error("Failed to send command:", error);
        });
}

function sendDelayCommand(delay) {
            setTimeout(() => {
                sendDelayValueCommand(delay);
            }, delay);
        }
// ฟังก์ชันสำหรับส่งคำสั่งการควบคุม Gripper ไปยัง ESP32
function sendDelayValueCommand(DelayValue) {

let  url = `http://${esp32IP}/delay?time=${DelayValue}`
fetch(url, {
    method: 'GET',
    mode: 'no-cors'
}).then(response => {
    console.log("Command sent successfully!");
}).catch(error => {
    console.error("Failed to send command:", error);
});
}
// ฟังก์ชันสำหรับส่งคำสั่งการควบคุม Gripper ไปยัง ESP32
function sendOutputValueCommand(input,state) {

let  url = `http://${esp32IP}/output?io=${input}&state=${state}`
fetch(url, {
    method: 'GET',
    mode: 'no-cors'
}).then(response => {
    console.log("Command sent successfully!");
}).catch(error => {
    console.error("Failed to send command:", error);
});
}

// ฟังก์ชันสำหรับส่งคำสั่งการควบคุม Gripper ไปยัง ESP32
function sendiftValueCommand(input,state) {

let  url = `http://${esp32IP}/if?io=${input}&state=${state}`
fetch(url, {
    method: 'GET',
    mode: 'no-cors'
}).then(response => {
    console.log("Command sent successfully!");
}).catch(error => {
    console.error("Failed to send command:", error);
});
}
function sendzeroValueCommand() {

let  url = `http://${esp32IP}/zero`
fetch(url, {
    method: 'GET',
    mode: 'no-cors'
}).then(response => {
    console.log("Command sent successfully!");
}).catch(error => {
    console.error("Failed to send command:", error);
});
}

function sendhomeValueCommand() {

let  url = `http://${esp32IP}/home`
fetch(url, {
    method: 'GET',
    mode: 'no-cors'
}).then(response => {
    console.log("Command sent successfully!");
}).catch(error => {
    console.error("Failed to send command:", error);
});
}
// ฟังก์ชันสำหรับดำเนินการตาม Object ที่แปลงมาจาก XML
async function executeCommands(commandObject) {
    if (!commandObject) return;
    for(let i = 0;i< commandObject.length;i++){
        console.log(commandObject[i]);
           switch (commandObject[i].type) {
        case 'move_arm':
           
            ARM_MODE =commandObject[i].fields.MODE;
            //const { theta1, theta2, theta3, speed, acceleration } = commandObject.values;
            //sendMoveCommand(theta1, theta2, theta3, speed, acceleration);
            break;
        case 'joint_angles':
          
            const { THETA1, THETA2, THETA3, SPEED, ACCELERATION } = commandObject[i].fields;
            await  sendMoveCommand(THETA1, THETA2, THETA3, SPEED, ACCELERATION,ARM_MODE);
            break;
        case 'inverse_kinematics':
    
            const { X, Y, Z, t, a } = commandObject[i].fields;
            await sendMoveCommand(commandObject[i].fields.X, commandObject[i].fields.Y, commandObject[i].fields.Z, commandObject[i].fields.SPEED, commandObject[i].fields.ACCELERATION,ARM_MODE);
            break;
        case 'delay_block':
            const { DELAY } = commandObject[i].fields;
            await sendDelayCommand(DELAY);
            return; // หยุดการดำเนินการจนกว่าจะครบเวลาหน่วงเวลา

        case 'gripper_value_control':
            await sendGripperValueCommand(commandObject[i].fields.GRIPPER);
            break;

        case 'gripper_on_off_control':
            const gripperValue = commandObject[i].fields.GRIPPER_STATE === "OPEN" ? 0 : 180;
            await sendGripperValueCommand(gripperValue);
            break;
        case 'set_zero':
            await sendzeroValueCommand();
            break;   
        case 'home_position':
            await sendhomeValueCommand();
            break;  
        case 'controls_for':
            await sendForCommand(commandObject[i].inputs.BY.fields.INT,commandObject[i].inputs.FROM.fields.INT,commandObject[i].inputs.TO.fields.INT);
            break;  //   
        case 'controls_if_high_low':
                // ตรวจสอบค่าที่กำหนดว่าเป็น HIGH หรือ LOW
                let state = commandObject[i].fields.STATE;
                let input = commandObject[i].inputs.INPUT.fields.INT ; // รับค่า input
                sendiftValueCommand(input,state);
                // แปลง HIGH เป็น 1 และ LOW เป็น 0 เพื่อให้ตรงกับเงื่อนไข
                let conditionMet = (state === 'HIGH' && input === 1) || (state === 'LOW' && input === 0);

                // ถ้าเงื่อนไขตรง ให้ทำการ execute บล็อก 'DO'
                if (conditionMet) {
                    await executeCommands(commandObject[i].inputs.DO);
                }
            break;  // 
        case 'output_block':
            
            await sendOutputValueCommand(commandObject[i].inputs.OUTPUT_VALUE.fields.INT,commandObject[i].fields.STATE);
            break;  //                           
        case 'controls_repeat_ext':
            let times = commandObject[i].inputs.TIMES.fields.INT;
            let DO = commandObject[i].inputs.DO;
                for (let o = 0; o < times; o++) {
                    console.log("loop");
                    console.log(DO);
                    await executeCommands(DO);
                }
                break;
            break;  //    
         case 'controls_repeat_boolean':
                // รับค่าเงื่อนไขที่กำหนดว่าเป็น TRUE หรือ FALSE
                 let condition = commandObject[i].fields.CONDITION === "TRUE";
                 let DO1 = commandObject[i].inputs.DO;
                // // วนลูปตราบใดที่ condition ยังคงเป็น TRUE
                while (condition) {
                    console.log("loop->");
                    console.log(DO1);
                    await executeCommands(DO1);
                    // ตรวจสอบเงื่อนไขทุกครั้งหลังจากดำเนินการ เพื่ออัปเดตการวนลูป
                    condition = commandObject[i].fields.CONDITION === "TRUE"; 
                }     
            break;  //                      
        default:
           // console.warn("Unknown command type:", commandObject.type);
    }

    //ดำเนินการบล็อกถัดไป
    executeCommands(commandObject.next);
    }
    
 
}

    </script>
</body>
</html>
