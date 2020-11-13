(%i1) load("cform.lisp");

/*===================================================================================*/
(%i2) cform(cos(t_)*(radius_ + a_/cosh(phi_ - omega_*t_)) + (a_*omega_*sinh(phi_ - omega_*t_)*sin(t_))/cosh(phi_ - omega_*t_)^2);
cos(t_)*(a_*pow(cosh(omega_*t_-phi_),-1)+radius_)-a_*omega_*sin(t_)*
    pow(cosh(omega_*t_-phi_),-2)*sinh(omega_*t_-phi_);
/*===================================================================================*/

/*===================================================================================*/
(%i3) cform((2*a_*omega_^2*sinh(phi_ - omega_*t_)^2*sin(t_))/cosh(phi_ - omega_*t_)^3 - (a_*omega_^2*sin(t_))/cosh(phi_ - omega_*t_) - sin(t_)*(radius_ + a_/cosh(phi_ - omega_*t_)) + (2*a_*omega_*sinh(phi_ - omega_*t_)*cos(t_))/cosh(phi_ - omega_*t_)^2);
2*a_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_),-3)*pow(sinh(omega_*
    t_-phi_),2)-2*a_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_),-2)*sinh(omega_*
    t_-phi_)-sin(t_)*(a_*pow(cosh(omega_*t_-phi_),-1)+radius_)-a_*pow(omega_,2)*
    sin(t_)*pow(cosh(omega_*t_-phi_),-1);
/*===================================================================================*/

/*===================================================================================*/
(%i4) cform((6*a_*omega_^2*sinh(phi_ - omega_*t_)^2*cos(t_))/cosh(phi_ - omega_*t_)^3 - (3*a_*omega_^2*cos(t_))/cosh(phi_ - omega_*t_) - cos(t_)*(radius_ + a_/cosh(phi_ - omega_*t_)) + (6*a_*omega_^3*sinh(phi_ - omega_*t_)^3*sin(t_))/cosh(phi_ - omega_*t_)^4 - (3*a_*omega_*sinh(phi_ - omega_*t_)*sin(t_))/cosh(phi_ - omega_*t_)^2 - (5*a_*omega_^3*sinh(phi_ - omega_*t_)*sin(t_))/cosh(phi_ - omega_*t_)^2);
(-6*a_*pow(omega_,3)*sin(t_)*pow(cosh(omega_*t_-phi_),-4)*pow(sinh(omega_*
    t_-phi_),3))+6*a_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_),-3)*
    pow(sinh(omega_*t_-phi_),2)+5*a_*pow(omega_,3)*sin(t_)*pow(cosh(omega_*
    t_-phi_),-2)*sinh(omega_*t_-phi_)+3*a_*omega_*sin(t_)*pow(cosh(omega_*
    t_-phi_),-2)*sinh(omega_*t_-phi_)-cos(t_)*(a_*pow(cosh(omega_*
    t_-phi_),-1)+radius_)-3*a_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*
    t_-phi_),-1);
/*===================================================================================*/
