/* Created by Language version: 7.5.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__GP
#define _nrn_initial _nrn_initial__GP
#define nrn_cur _nrn_cur__GP
#define _nrn_current _nrn_current__GP
#define nrn_jacob _nrn_jacob__GP
#define nrn_state _nrn_state__GP
#define _net_receive _net_receive__GP 
#define evaluate_fct evaluate_fct__GP 
#define states states__GP 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define enaD _p[0]
#define ekD _p[1]
#define ecaD _p[2]
#define gnabar _p[3]
#define gkdrbar _p[4]
#define gl _p[5]
#define el _p[6]
#define gcatbar _p[7]
#define gahp _p[8]
#define inaD _p[9]
#define ikD _p[10]
#define icaD _p[11]
#define icaDT _p[12]
#define ilk _p[13]
#define iahp _p[14]
#define h_inf _p[15]
#define tau_h _p[16]
#define m_inf _p[17]
#define n_inf _p[18]
#define tau_n _p[19]
#define s_inf _p[20]
#define r_inf _p[21]
#define a_inf _p[22]
#define h _p[23]
#define n _p[24]
#define r _p[25]
#define CA _p[26]
#define Dh _p[27]
#define Dn _p[28]
#define Dr _p[29]
#define DCA _p[30]
#define v _p[31]
#define _g _p[32]
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_evaluate_fct(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_GP", _hoc_setdata,
 "evaluate_fct_GP", _hoc_evaluate_fct,
 0, 0
};
 /* declare global and static user variables */
#define gt gt_GP
 double gt = 0.0005;
#define k2_ca k2_ca_GP
 double k2_ca = 0.015;
#define k1_ca k1_ca_GP
 double k1_ca = 0.1;
#define k_a k_a_GP
 double k_a = -2;
#define k_r k_r_GP
 double k_r = 2;
#define k_s k_s_GP
 double k_s = -2;
#define k_n k_n_GP
 double k_n = -14;
#define k_h k_h_GP
 double k_h = 12;
#define k_m k_m_GP
 double k_m = -10;
#define sig_n2 sig_n2_GP
 double sig_n2 = -12;
#define sig_h2 sig_h2_GP
 double sig_h2 = -12;
#define theta_a theta_a_GP
 double theta_a = -57;
#define tau_r tau_r_GP
 double tau_r = 30;
#define theta_r theta_r_GP
 double theta_r = -70;
#define theta_s theta_s_GP
 double theta_s = -35;
#define tht_n2 tht_n2_GP
 double tht_n2 = -40;
#define tau_n1 tau_n1_GP
 double tau_n1 = 0.27;
#define tau_n0 tau_n0_GP
 double tau_n0 = 0.05;
#define theta_n theta_n_GP
 double theta_n = -50;
#define tht_h2 tht_h2_GP
 double tht_h2 = -40;
#define tau_h1 tau_h1_GP
 double tau_h1 = 0.27;
#define tau_h0 tau_h0_GP
 double tau_h0 = 0.05;
#define theta_h theta_h_GP
 double theta_h = -58;
#define theta_m theta_m_GP
 double theta_m = -37;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "theta_m_GP", "mV",
 "theta_h_GP", "mV",
 "k_m_GP", "mV",
 "k_h_GP", "mV",
 "tau_h0_GP", "ms",
 "tau_h1_GP", "ms",
 "tht_h2_GP", "mV",
 "sig_h2_GP", "mV",
 "theta_n_GP", "mV",
 "k_n_GP", "mV",
 "tau_n0_GP", "ms",
 "tau_n1_GP", "ms",
 "tht_n2_GP", "mV",
 "sig_n2_GP", "mV",
 "theta_s_GP", "mV",
 "k_s_GP", "mV",
 "gt_GP", "S/cm2",
 "theta_r_GP", "mV",
 "k_r_GP", "mV",
 "tau_r_GP", "ms",
 "theta_a_GP", "mV",
 "k_a_GP", "mV",
 "k1_ca_GP", "cm2/mA/ms",
 "k2_ca_GP", "mA/cm2",
 "enaD_GP", "mV",
 "ekD_GP", "mV",
 "ecaD_GP", "mV",
 "gnabar_GP", "S/cm2",
 "gkdrbar_GP", "S/cm2",
 "gl_GP", "S/cm2",
 "el_GP", "mV",
 "gcatbar_GP", "S/cm2",
 "gahp_GP", "S/cm2",
 "inaD_GP", "mA/cm2",
 "ikD_GP", "mA/cm2",
 "icaD_GP", "mA/cm2",
 "icaDT_GP", "mA/cm2",
 "ilk_GP", "mA/cm2",
 "iahp_GP", "mA/cm2",
 "tau_h_GP", "ms",
 "tau_n_GP", "ms",
 0,0
};
 static double CA0 = 0;
 static double delta_t = 0.01;
 static double h0 = 0;
 static double n0 = 0;
 static double r0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "theta_m_GP", &theta_m_GP,
 "theta_h_GP", &theta_h_GP,
 "k_m_GP", &k_m_GP,
 "k_h_GP", &k_h_GP,
 "tau_h0_GP", &tau_h0_GP,
 "tau_h1_GP", &tau_h1_GP,
 "tht_h2_GP", &tht_h2_GP,
 "sig_h2_GP", &sig_h2_GP,
 "theta_n_GP", &theta_n_GP,
 "k_n_GP", &k_n_GP,
 "tau_n0_GP", &tau_n0_GP,
 "tau_n1_GP", &tau_n1_GP,
 "tht_n2_GP", &tht_n2_GP,
 "sig_n2_GP", &sig_n2_GP,
 "theta_s_GP", &theta_s_GP,
 "k_s_GP", &k_s_GP,
 "gt_GP", &gt_GP,
 "theta_r_GP", &theta_r_GP,
 "k_r_GP", &k_r_GP,
 "tau_r_GP", &tau_r_GP,
 "theta_a_GP", &theta_a_GP,
 "k_a_GP", &k_a_GP,
 "k1_ca_GP", &k1_ca_GP,
 "k2_ca_GP", &k2_ca_GP,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[0]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.5.0",
"GP",
 "enaD_GP",
 "ekD_GP",
 "ecaD_GP",
 "gnabar_GP",
 "gkdrbar_GP",
 "gl_GP",
 "el_GP",
 "gcatbar_GP",
 "gahp_GP",
 0,
 "inaD_GP",
 "ikD_GP",
 "icaD_GP",
 "icaDT_GP",
 "ilk_GP",
 "iahp_GP",
 "h_inf_GP",
 "tau_h_GP",
 "m_inf_GP",
 "n_inf_GP",
 "tau_n_GP",
 "s_inf_GP",
 "r_inf_GP",
 "a_inf_GP",
 0,
 "h_GP",
 "n_GP",
 "r_GP",
 "CA_GP",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 33, _prop);
 	/*initialize range parameters*/
 	enaD = 55;
 	ekD = -80;
 	ecaD = 120;
 	gnabar = 0.12;
 	gkdrbar = 0.03;
 	gl = 0.0001;
 	el = -65;
 	gcatbar = 0.00015;
 	gahp = 0.01;
 	_prop->param = _p;
 	_prop->param_size = 33;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 1, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _GP_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
  hoc_register_prop_size(_mechtype, 33, 1);
  hoc_register_dparam_semantics(_mechtype, 0, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 GP /home/jhielson/catkin_ws/src/neural_model/scripts/x86_64/GP.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "All ion channels used in GP models";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int evaluate_fct(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   evaluate_fct ( _threadargscomma_ v ) ;
   Dn = 0.1 * ( n_inf - n ) / tau_n ;
   Dh = 0.05 * ( h_inf - h ) / tau_h ;
   Dr = ( r_inf - r ) / tau_r ;
   DCA = k1_ca * ( - icaD - icaDT - k2_ca * CA ) ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 evaluate_fct ( _threadargscomma_ v ) ;
 Dn = Dn  / (1. - dt*( ( ( 0.1 )*( ( ( - 1.0 ) ) ) ) / tau_n )) ;
 Dh = Dh  / (1. - dt*( ( ( 0.05 )*( ( ( - 1.0 ) ) ) ) / tau_h )) ;
 Dr = Dr  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_r )) ;
 DCA = DCA  / (1. - dt*( ( k1_ca )*( ( ( - ( k2_ca )*( 1.0 ) ) ) ) )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   evaluate_fct ( _threadargscomma_ v ) ;
    n = n + (1. - exp(dt*(( ( 0.1 )*( ( ( - 1.0 ) ) ) ) / tau_n)))*(- ( ( ( 0.1 )*( ( n_inf ) ) ) / tau_n ) / ( ( ( 0.1 )*( ( ( - 1.0 ) ) ) ) / tau_n ) - n) ;
    h = h + (1. - exp(dt*(( ( 0.05 )*( ( ( - 1.0 ) ) ) ) / tau_h)))*(- ( ( ( 0.05 )*( ( h_inf ) ) ) / tau_h ) / ( ( ( 0.05 )*( ( ( - 1.0 ) ) ) ) / tau_h ) - h) ;
    r = r + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_r)))*(- ( ( ( r_inf ) ) / tau_r ) / ( ( ( ( - 1.0 ) ) ) / tau_r ) - r) ;
    CA = CA + (1. - exp(dt*(( k1_ca )*( ( ( - ( k2_ca )*( 1.0 ) ) ) ))))*(- ( ( k1_ca )*( ( - icaD - icaDT ) ) ) / ( ( k1_ca )*( ( ( - ( k2_ca )*( 1.0 ) ) ) ) ) - CA) ;
   }
  return 0;
}
 
static int  evaluate_fct ( _threadargsprotocomma_ double _lv ) {
   h_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_h ) / k_h ) ) ;
   m_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_m ) / k_m ) ) ;
   tau_h = tau_h0 + tau_h1 / ( 1.0 + exp ( - ( _lv - tht_h2 ) / sig_h2 ) ) ;
   n_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_n ) / k_n ) ) ;
   tau_n = tau_n0 + tau_n1 / ( 1.0 + exp ( - ( _lv - tht_n2 ) / sig_n2 ) ) ;
   s_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_s ) / k_s ) ) ;
   r_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_r ) / k_r ) ) ;
   a_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_a ) / k_a ) ) ;
    return 0; }
 
static void _hoc_evaluate_fct(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 evaluate_fct ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 4; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  CA = CA0;
  h = h0;
  n = n0;
  r = r0;
 {
   evaluate_fct ( _threadargscomma_ v ) ;
   h = h_inf ;
   n = n_inf ;
   r = r_inf ;
   CA = 0.1 ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   inaD = gnabar * m_inf * m_inf * m_inf * h * ( v - enaD ) ;
   ikD = gkdrbar * pow( n , 4.0 ) * ( v - ekD ) ;
   icaDT = gt * a_inf * a_inf * a_inf * r * ( v - ecaD ) ;
   ilk = gl * ( v - el ) ;
   icaD = gcatbar * s_inf * s_inf * ( v - ecaD ) ;
   iahp = gahp * ( v - ekD ) * ( CA / ( CA + 10.0 ) ) ;
   }
 _current += ilk;
 _current += inaD;
 _current += ikD;
 _current += icaDT;
 _current += icaD;
 _current += iahp;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   states(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(n) - _p;  _dlist1[0] = &(Dn) - _p;
 _slist1[1] = &(h) - _p;  _dlist1[1] = &(Dh) - _p;
 _slist1[2] = &(r) - _p;  _dlist1[2] = &(Dr) - _p;
 _slist1[3] = &(CA) - _p;  _dlist1[3] = &(DCA) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
