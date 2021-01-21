function [posVO, orientVO, paramsVO] ...
    = helperVisualOdometryModel(pos, orient, paramsVO)

%  获得模型参数
scaleVO = paramsVO.scale;
sigmaN = paramsVO.sigmaN;
tau = paramsVO.tau;
sigmaB = paramsVO.sigmaB;
sigmaA = sqrt((2/tau) + 1/(tau*tau))*sigmaB;
b = paramsVO.driftBias;

% 计算漂移
b = (1 - 1/tau).*b + randn(1,3)*sigmaA; % 漂移
drift = randn(1,3)*sigmaN + b;
paramsVO.driftBias = b;

% Calculate visual odometry measurements.
posVO = scaleVO*pos + drift;
orientVO = orient;
end