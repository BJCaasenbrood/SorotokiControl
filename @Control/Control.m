classdef Control < handle

    properties (Access = public)
        t;
        U;      % control signal
        Y;      % measurement signal
        P0;     %
        Ip;
        Usr;
        Pwd;
        Port;
        Log;
        Sensor;
        Actuator;
        Frequency;
        NVeab;
    end
    
    properties (Access = private)
        out;
        Ndev;
        pyFile;
        outFile;
        conFile;
        autoConnect;
        autoCompile;
        autoTransfer;
        autoi2cCheck;
        isConnected;
        i2cBusses;
        SSH;
        TCPclient;
        LoopCounter;
        LoopIndex;
    end
    
%--------------------------------------------------------------------------
methods  
%-------------------------------------------------------- ballong-dog class 
function obj = Control(usr,ip,pwd,varargin)
    
    obj.Port = 8888;
    obj.Ip  = ip;
    obj.Usr = usr;
    obj.Pwd = pwd;
    
    obj.t       = 0;
    obj.Ndev    = 0;
    obj.pyFile  = 'runme.py';
    obj.outFile = 'soro.log';
    obj.conFile = 'config.txt';
    obj.autoCompile = false;
    obj.autoConnect = false;
    obj.i2cBusses   = [1,3,4,5,6];
    obj.isConnected = false;
    obj.autoConnect = true;
    obj.LoopCounter = 0;
    obj.LoopIndex = uint8(1e6);
    obj.Frequency = 30;
    obj.Log = struct('y',[],'t',[]);
    obj.NVeab = 1;
    
    for ii = 1:2:length(varargin)
        obj.(varargin{ii}) = varargin{ii+1};
    end
    
    if isempty(obj.P0)
        obj.P0 = zeros(2*obj.NVeab,1);
    end
    
    obj.U = pdac(obj.P0);
    
    % setup TCP client
    %client = tcpip(obj.Ip,obj.Port,'NetworkRole', 'client');
    %echotcpip("on",obj.Port)
    
    client = tcpclient(obj.Ip,obj.Port);
    client.ByteOrder = 'little-endian';
    obj.TCPclient = client;

    if obj.autoConnect
        obj = connect(obj); 
    end
end
%---------------------------------------------------------------------- get     
function varargout = get(Control,varargin)
    if nargin > 1
        varargout{nargin-1,1} = [];
        for ii = 1:length(varargin)
            varargout{ii,1} = Control.(varargin{ii});
        end
    else
        varargout = Control.(varargin);
    end
end     
%---------------------------------------------------------------------- set
function Control = set(Control,varargin)
    for ii = 1:2:length(varargin)
        Control.(varargin{ii}) = varargin{ii+1};
    end
end
%---------------------------------------------------------------------- set
function Control = setInput(Control,x,varargin)
    
    if (numel(x) == 1) && ~isempty(varargin)
       X = Control.U*0;
       x(round(x)) = varargin{1};
    end

    Control.U = clamp(pdac(x(:) - Control.P0(:)),0,1);
end
%---------------------------------------------------------------------- set
function y = getOutput(Control)
   y = padc(Control.Log.y(end,:)); 
end
%---------------------------------------------------------------------- set
function resetInput(Control)
    Control.U = clamp(pdac(zeros(Control.NVeab*2,1) + Control.P0(:)),0,1);
    Control.tcpSendData(Control.U);
end

%--------------------------------------------------------- connect to board
function Control = connect(Control)

    if ~Control.autoConnect
        str = action({'(y)es, continue with SSH connection',...
            '(n)o, stop connection'},'s');
        
        if strcmp(str,'n'), return;
        elseif strcmp(str,'y'), pause(0.1);
        else, error('terminated');
        end
    end
    
    ip_   = Control.Ip;
    usr_  = Control.Usr;
    pwd_  = Control.Pwd;
    fprintf('Starting SSH server \n');
    fprintf('Establishing connection to target-pc: \n    dev -- ');
    fprintf(['',Control.Usr, '@' , Control.Ip,': ...']); pause(0.25);
    
    SSH_ = ssh2_config(ip_,usr_,pwd_);
    Control.SSH = ssh2(SSH_);
    
    % cout('\b\b\b');
    % cout('green','Connected!\n');
    fprintf('Connected!\n');
    Control.isConnected = true;

    %if checkexist(Control,Control.cppFile) && Control.autoCompile
    %    str = action({'(y)es, recompile executable file',...
    %            '(n)o'},'s');
    %     CallExecuted([FILENAME,' found!']);
    %     request = CallRequest('recompile executable file?','y/n');
    %     if strcmp(request,'y')
    %         brd = CommandShell(brd,'chmod 755 Soro*',0);
    %         CallExecuted(['compiled ',FILENAME,'!']);
    %     end
    %     if existsFile(brd,'soro.log')
    %         CallExecuted('cleared log file');
    %         brd = CommandShell(brd,'rm soro.log',0);
    %     end
    % else
    %     CallExecuted([FILENAME,' not found! File is required!'])
    % end
    
end
%--------------------------------------------------------------- disconnect   
function Control = disconnect(Control)
    fprintf('Closing connection with target-pc: \n    dev -- '); 
    fprintf(['',Control.Usr, '@' , Control.Ip,': ...']); pause(0.25);
    
    Control.SSH = ssh2_close(Control.SSH);
    %Control.SSH = [];
    fprintf('Closed!\n');
end
%------------------------------ add sensor or actuator to device on i2c bus   
function Control = addDevice(Control,varargin)
   
    for ii = 1:3:length(varargin)
        
        i2cbus = dev2i2c(varargin{ii+1});
        ID = {vertcat(i2cbus{:}),(varargin{ii+2})};
        
        if Control.isConnected && Control.autoi2cCheck
            Control = Control.shell(['python3 i2cscan.py ',...
                num2str(varargin{ii+2})]);
            
            if ismember(Control.out,horzcat(i2cbus{:}))
                fprintf(['    dev -- ',varargin{ii+1},...
                   ' found on i2c bus: ',num2str(varargin{ii+2}),'\n']);
            else
                fprintf(['    dev -- ',varargin{ii+1}, ...
                   ' not found on i2c bus: ',num2str(varargin{ii+2}),...
                   '\n \t   Perhaps check wiring, or wrong i2c port?\n']); 
            end
        end
        
        Control.(varargin{ii}) = [Control.(varargin{ii});ID];
        Control.Ndev = Control.Ndev  + 1;
    end
end
%------------------------------ send terminal command (with printed output)    
function Control = command(Control,Arg)
    [Control.SSH, Control.out] = ssh2_command(Control.SSH, Arg, 1);
end
%---------------------------------------- send terminal command (no output) 
function Control = shell(Control,Arg)
    [Control.SSH, Control.out] = ssh2_command(Control.SSH, Arg, 0);
end
%------------------------------------------------------ transfer file to Pi   
function Control = transfer(Control,Arg)
    Control.SSH = scp_simple_put(Control.Ip,Control.Usr,Control.Pwd,Arg);
end
%--------------------------------------------- transfer python script to Pi   
function Control = transfertoPi(Control)
    
    N = 1;
    % while loop for matlab to finish file generations
    while ~exist(Control.pyFile,'file') && (N < 500)
       N = N + 1;
       pause(0.1);
    end
    
    Control.SSH = scp_simple_put(Control.ip, ... 
        Control.usr, ...
        Control.pwd, ...
        Control.pyFile);
end
%-------------------------- Send data as a TCP client (formatted as double)
function Control = tcpSendData(Control, data)

    %dU = pdac(Control.P0) - 0.5;

    x = clamp(data,0,1);
    %fwrite(Control.TCPclient,clamp(x(:),0,1),"double");
    Control.TCPclient.write(clamp(x(:),0,1),"double");
    %fwrite(Control.TCPclient,clamp(x(:),0,1),"double");
end
%-------------------------- Recv data as a TCP client (formatted as double)
function data = tcpRecvData(Control,data_length)
    
    data = zeros(1,Control.NVeab*2);
    for ii = 1:(Control.NVeab*2)
        %data(ii) = fread(Control.TCPclient,data_length,"double");
        %out = Control.TCPclient.read(data_length,"double")
        %try
            data(ii) = Control.TCPclient.read(1,"double");
            %data(ii) = fread(Control.TCPclient,data_length,"double");
        %catch

        %end
    end

    flushoutput(Control.TCPclient);  % flush all outputs    
    %data1 = fread(Control.TCPclient,data_length,"double");
    %data2 = fread(Control.TCPclient,data_length,"double");
    
    %data = [data1,data2];
    Yn = padc(data(:).') - Control.P0(:).';
    Control.Log.y = [Control.Log.y; Yn(:).'];
    
    Control.t = (1/Control.Frequency)*Control.LoopCounter;
    Control.Log.t = [Control.Log.t; Control.t];
end
%----------------------------------------------------------- run executable 
function flag = loop(Control,Ts)
    
    if Control.LoopCounter == 0
        %fopen(Control.TCPclient);

        tic;
        % fprintf('Waiting for VEABs to calibrate, please wait...\n');
        % while toc<3
        %      tcpRecvData(Control,1);
        %      Control.tcpSendData(Control.U);
        %      pause(0.01);
        % end
        % 
        % Poffset = mean(Control.Log.y,1);
        % Control.P0 = Poffset;
        % Control.Log.y = [];
        % Control.Log.t = [];

        tic;
        Control.LoopCounter = 1;
        Control.LoopIndex   = Ts / (1/Control.Frequency);
        %tcpRecvData(Control,10);
        Control.tcpSendData(Control.U);
    else
        
        tcpRecvData(Control,10);
        Control.tcpSendData(Control.U);
        
        while toc < 1/Control.Frequency
            continue
        end
% %         
        tic;
        Control.LoopCounter = Control.LoopCounter + 1; 
        Control.t = (1/Control.Frequency) * Control.LoopCounter;
    end
    
    flushinput(Control.TCPclient);	  % flush all inputs
    flushoutput(Control.TCPclient);  % flush all outputs
    
    if Control.LoopCounter <= Control.LoopIndex
        flag = true;
    else
        flag = false;
    end
    
end
%----------------------------------------------------------- run executable 
function reset(Control)
    tcpSendData(Control,0);
end
%--------------------------------------------- reads log file, return array
function A = read(Control,filename)
    if checkexist(Control,filename)
        Control = shell(Control,['tail ',Control.outFile,' -n 50000']);
        A = (cellfun(@str2num,Control.out(1:end),'un',0));
        A = vertcat(A{:});
    else
        A = [];
        fprintf(' log file does not exist!\n');
    end
end
%----------------------------------------------------- check if file exists  
function bool = checkexist(Control,name)
    Control = shell(Control,['[ -f ',name,' ] && echo "1" || echo "0"']);
    bool = str2double(Control.out{end});
end
end

methods (Access = private)

end
end
