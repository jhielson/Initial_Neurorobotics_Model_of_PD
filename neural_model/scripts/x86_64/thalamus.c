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
 
#define nrn_init _nrn_init__thalamus
#define _nrn_initial _nrn_initial__thalamus
#define nrn_cur _nrn_cur__thalamus
#define _nrn_current _nrn_current__thalamus
#define nrn_jacob _nrn_jacob__thalamus
#define nrn_state _nrn_state__thalamus
#define _net_receive _net_receive__thalamus 
#define evaluate_fct evaluate_fct__thalamus 
#define states states__thalamus 
 
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
#define ena _p[0]
#define ek _p[1]
#define et _p[2]
#define gnabar _p[3]
#define gkdrbar _p[4]
#define gl _p[5]
#define el _p[6]
#define ina_th _p[7]
#define ik_th _p[8]
#define it _p[9]
#define ilk _p[10]
#define h_inf _p[11]
#define tau_h _p[12]
#define m_inf _p[13]
#define p_inf _p[14]
#define r_inf _p[15]
#define h _p[16]
#define r _p[17]
#define tau_r _p[18]
#define Dh _p[19]
#define Dr _p[20]
#define v _p[21]
#define _g _p[22]
 
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
 "setdata_thalamus", _hoc_setdata,
 "evaluate_fct_thalamus", _hoc_evaluate_fct,
 0, 0
};
 /* declare global and static user variables */
#define gt gt_thalamus
 double gt = 0.005;
#define k_r k_r_thalamus
 double k_r = 4;
#define k_p k_p_thalamus
 double k_p = -6.2;
#define k_h1 k_h1_thalamus
 double k_h1 = -5;
#define k_h0 k_h0_thalamus
 double k_h0 = -18;
#define k_h k_h_thalamus
 double k_h = 4;
#define k_m k_m_thalamus
 double k_m = -7;
#define tau_r1 tau_r1_thalamus
 double tau_r1 = -10.5;
#define th_r1 th_r1_thalamus
 double th_r1 = -25;
#define theta_r theta_r_thalamus
 double theta_r = -84;
#define theta_p theta_p_thalamus
 double theta_p = -60;
#define theta_h1 theta_h1_thalamus
 double theta_h1 = -23;
#define tau_h1 tau_h1_thalamus
 double tau_h1 = 4;
#define theta_h0 theta_h0_thalamus
 double theta_h0 = -46;
#define tau_h0 tau_h0_thalamus
 double tau_h0 = 0.128;
#define theta_h theta_h_thalamus
 double theta_h = -41;
#define theta_m theta_m_thalamus
 double theta_m = -37;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "theta_m_thalamus", "mV",
 "theta_h_thalamus", "mV",
 "k_m_thalamus", "mV",
 "k_h_thalamus", "mV",
 "tau_h0_thalamus", "1/ms",
 "theta_h0_thalamus", "mV",
 "k_h0_thalamus", "mV",
 "tau_h1_thalamus", "1/ms",
 "theta_h1_thalamus", "mV",
 "k_h1_thalamus", "mV",
 "theta_p_thalamus", "mV",
 "k_p_thalamus", "mV",
 "gt_thalamus", "S/cm2",
 "theta_r_thalamus", "mV",
 "k_r_thalamus", "mV",
 "th_r1_thalamus", "mV",
 "tau_r1_thalamus", "mV",
 "ena_thalamus", "mV",
 "ek_thalamus", "mV",
 "et_thalamus", "mV",
 "gnabar_thalamus", "S/cm2",
 "gkdrbar_thalamus", "S/cm2",
 "gl_thalamus", "S/cm2",
 "el_thalamus", "mV",
 "ina_th_thalamus", "mA/cm2",
 "ik_th_thalamus", "mA/cm2",
 "it_thalamus", "mA/cm2",
 "ilk_thalamus", "mA/cm2",
 "tau_h_thalamus", "ms",
 0,0
};
 static double delta_t = 0.01;
 static double h0 = 0;
 static double r0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "theta_m_thalamus", &theta_m_thalamus,
 "theta_h_thalamus", &theta_h_thalamus,
 "k_m_thalamus", &k_m_thalamus,
 "k_h_thalamus", &k_h_thalamus,
 "tau_h0_thalamus", &tau_h0_thalamus,
 "theta_h0_thalamus", &theta_h0_thalamus,
 "k_h0_thalamus", &k_h0_thalamus,
 "tau_h1_thalamus", &tau_h1_thalamus,
 "theta_h1_thalamus", &theta_h1_thalamus,
 "k_h1_thalamus", &k_h1_thalamus,
 "theta_p_thalamus", &theta_p_thalamus,
 "k_p_thalamus", &k_p_thalamus,
 "gt_thalamus", &gt_thalamus,
 "theta_r_thalamus", &theta_r_thalamus,
 "k_r_thalamus", &k_r_thalamus,
 "th_r1_thalamus", &th_r1_thalamus,
 "tau_r1_thalamus", &tau_r1_thalamus,
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
"thalamus",
 "ena_thalamus",
 "ek_thalamus",
 "et_thalamus",
 "gnabar_thalamus",
 "gkdrbar_thalamus",
 "gl_thalamus",
 "el_thalamus",
 0,
 "ina_th_thalamus",
 "ik_th_thalamus",
 "it_thalamus",
 "ilk_thalamus",
 "h_inf_thalamus",
 "tau_h_thalamus",
 "m_inf_thalamus",
 "p_inf_thalamus",
 "r_inf_thalamus",
 0,
 "h_thalamus",
 "r_thalamus",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 23, _prop);
 	/*initialize range parameters*/
 	ena = 50;
 	ek = -75;
 	et = 0;
 	gnabar = 0.003;
 	gkdrbar = 0.005;
 	gl = 5e-05;
 	el = -70;
 	_prop->param = _p;
 	_prop->param_size = 23;
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

 void _thalamus_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
  hoc_register_prop_size(_mechtype, 23, 1);
  hoc_register_dparam_semantics(_mechtype, 0, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 thalamus /home/jhielson/catkin_ws/src/neural_model/scripts/x86_64/thalamus.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "All ion channels used in thalamic models";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int evaluate_fct(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[2], _dlist1[2];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   evaluate_fct ( _threadargscomma_ v ) ;
   Dh = ( h_inf - h ) / tau_h ;
   Dr = ( r_inf - r ) / tau_r ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 evaluate_fct ( _threadargscomma_ v ) ;
 Dh = Dh  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_h )) ;
 Dr = Dr  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_r )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   evaluate_fct ( _threadargscomma_ v ) ;
    h = h + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_h)))*(- ( ( ( h_inf ) ) / tau_h ) / ( ( ( ( - 1.0 ) ) ) / tau_h ) - h) ;
    r = r + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_r)))*(- ( ( ( r_inf ) ) / tau_r ) / ( ( ( ( - 1.0 ) ) ) / tau_r ) - r) ;
   }
  return 0;
}
 
static int  evaluate_fct ( _threadargsprotocomma_ double _lv ) {
   h_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_h ) / k_h ) ) ;
   m_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_m ) / k_m ) ) ;
   tau_h = 1.0 / ( tau_h0 * exp ( ( _lv - theta_h0 ) / k_h0 ) + tau_h1 / ( 1.0 + exp ( ( _lv - theta_h1 ) / k_h1 ) ) ) ;
   p_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_p ) / k_p ) ) ;
   r_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_r ) / k_r ) ) ;
   tau_r = 0.15 * ( 28.0 + exp ( ( _lv - th_r1 ) / tau_r1 ) ) ;
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
 
static int _ode_count(int _type){ return 2;}
 
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
	for (_i=0; _i < 2; ++_i) {
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
  h = h0;
  r = r0;
 {
   evaluate_fct ( _threadargscomma_ v ) ;
   h = h_inf ;
   r = r_inf ;
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
   ina_th = gnabar * m_inf * m_inf * m_inf * h * ( v - ena ) ;
   ik_th = gkdrbar * pow( ( 0.75 * ( 1.0 - h ) ) , 4.0 ) * ( v - ek ) ;
   ilk = gl * ( v - el ) ;
   it = gt * ( v - et ) * r * p_inf * p_inf ;
   }
 _current += ilk;
 _current += it;
 _current += ik_th;
 _current += ina_th;

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
 _slist1[0] = &(h) - _p;  _dlist1[0] = &(Dh) - _p;
 _slist1[1] = &(r) - _p;  _dlist1[1] = &(Dr) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
