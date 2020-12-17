%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Equations of motion function %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ydot = EOM(t,y,k0,k1,k2,k3,a,rho,A,D)

global n

pt = y(end-2:end);                             % target position

ptdot=[sin((2*pi/50)*t) cos((2*pi/50)*t) 0.75*sin((6*pi/50)*t)]';   % velocity of target (roller coaster)

%control law
Pa=eye(3,3)-a*a';   % orthogonal projection matrix of a
for i=1:n
    p_i = y(3*(i-1)+1:3*(i-1)+3);
    pit = p_i-pt;
    phii = pit/norm(pit);
    phiia = Pa*phii/norm(Pa*phii);
    DeliV1 = (a*a')*pit;
    DeliV2 = (norm(Pa*pit)-rho)*phiia;
    DeliV3 = 0;
    for j=1:n
        aij = A(i,j);
        dij = D(i,j);
        pj = y(3*(j-1)+1:3*(j-1)+3);
        pjt = pj-pt;
        phij = pjt/norm(pjt);
        phija = Pa*phij/norm(Pa*phij);
        gammaij = aij*(norm(phiia-phija)^2 - dij^2/rho^2)*...
                    cross(a,phiia)'*(phiia-phija);
        DeliV3 = DeliV3 + (1/norm(Pa*pit))*gammaij*cross(a,phiia);
    end
    ui1 = -k1*DeliV1;
    ui2 = -k2*DeliV2 + k0*norm(Pa*pit)*cross(a,phiia);
    ui3 = -k3*norm(Pa*pit)*DeliV3;
    ui = ui1 + ui2 + ui3 + ptdot;
    if i==1
        ydot = ui;
    else
        ydot = [ydot; ui];
    end
end

ydot = [ydot; ptdot];  % concatenate with target velocity
