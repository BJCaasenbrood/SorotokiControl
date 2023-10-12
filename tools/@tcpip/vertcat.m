function out = vertcat(varargin)
%VERTCAT Vertical concatenation of instrument objects.
%

%   MP 7-13-99
%   Copyright 1999-2009 The MathWorks, Inc. 

% Initialize variables.
c=[];

% Loop through each java object and concatenate.
for i = 1:nargin
    if ~isempty(varargin{i}),
        
        if isempty(c)
            % Must be an instrument object otherwise error.
            if ~isa(varargin{i},'instrument')
                error(message('instrument:vertcat:nonInstrumentConcat'))
            end
            c=varargin{i};
        else
          
            % Extract needed information.
            try
                appendJObject = varargin{i}.jobject;              
                appendConstructor = varargin{i}.constructor;
                appendStore = varargin{i}.store;
            catch
                % This will fail if not an instrument object.
                error(message('instrument:vertcat:nonInstrumentConcat'))
            end
            
            % Append the jobject field.
            try
                c.jobject = [c.jobject; appendJObject];
            catch aException
                rethrow(aException);
            end
            
			% Append the Constructor field.
            if ischar(c.constructor)
                if ischar(appendConstructor)
                    c.constructor = {c.constructor appendConstructor}';
                else
                    c.constructor = {c.constructor appendConstructor{:}}';
                end
            else
                if ischar(appendConstructor)
                    c.constructor = {c.constructor{:} appendConstructor}';
                else
                    c.constructor = {c.constructor{:} appendConstructor{:}}';
                end
            end
        end 
    end
end

% Verify that a matrix of objects was not created.
if length(c.jobject) ~= numel(c.jobject)
    error(message('instrument:vertcat:nonMatrixConcat'))
end

% Output the array of objects.
out = c;

