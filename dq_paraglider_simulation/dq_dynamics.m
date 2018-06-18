function dq_dynamics(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.

setup(block);

%endfunction

function setup(block)

% Register number of ports
block.NumInputPorts  = 2; % dual_omega[8] dual_q[8]
block.NumOutputPorts = 2; % dual_omega[8] dual_q[8]

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions  = [1 8];
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).Dimensions  = [1 8];
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions  = [1 8];
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

block.OutputPort(2).Dimensions  = [1 8];
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
% block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
% block.RegBlockMethod('InitializeConditions', @InitializeConditions);
% block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
% block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

function Outputs(block)

dual_omega = block.InputPort(1).Data;
dual_q     = block.InputPort(2).Data;

mass   = 1;     % Масса ЛА, [кг]

% -------------------------------------------------------------------------
% Тензор инерции
% -------------------------------------------------------------------------
Jq = [1  0       0        0;
      0  3.670   0        0;
      0  0       3.710    0;
      0  0       0        0.104];
mq = [1   0    0    0;
      0  mass  0    0;
      0   0   mass  0;
      0   0    0   mass]; 
  
dual_J = [  Jq       zeros(4); 
          zeros(4)     mq];

% -------------------------------------------------------------------------
      
dual_F = [0 0 0 0  0 0 0 0];

d_dual_omega_dt = (dual_J\dual_F' - dual_J\dq_cross([dual_omega(1:4) 0 0 0 0], (dual_J*dual_omega')')' )'; 
d_dual_q_dt     = 0.5*dq_multiply(dual_q, dual_omega);
d_dual_q_dt
block.OutputPort(1).Data = d_dual_omega_dt;
block.OutputPort(2).Data = d_dual_q_dt;

%end Outputs

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  for i = 1:block.NumOutputPorts
    block.OutputPort(i).SamplingMode  = fd;    
  end
  
  
%endfunction

function Terminate(block)

%end Terminate

