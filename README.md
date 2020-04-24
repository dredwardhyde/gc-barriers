# Implementation changes in Shenandoah GC barriers in JDK 13

All garbage collector barriers must be derived from **BarrierSetAssembler** class which implements default barriers. Then custom barrier class must be installed during heap initialization using **BarrierSet::set_barrier_set** method:
```cpp
ShenandoahBarrierSet::ShenandoahBarrierSet(ShenandoahHeap* heap) :
  BarrierSet(make_barrier_set_assembler<ShenandoahBarrierSetAssembler>(),
             make_barrier_set_c1<ShenandoahBarrierSetC1>(),
             make_barrier_set_c2<ShenandoahBarrierSetC2>(),
             NULL /* barrier_set_nmethod */,
             BarrierSet::FakeRtti(BarrierSet::ShenandoahBarrierSet)),
  _heap(heap),
  _satb_mark_queue_set(){}
  
....

BarrierSet::set_barrier_set(new ShenandoahBarrierSet(this));
```

Class **BarrierSetAssembler** contains multiple methods, each one represents point at which custom GC barrier could be inserted:
```cpp
class BarrierSetAssembler: public CHeapObj<mtGC> {
...
public:
  virtual void arraycopy_prologue(...) {}
  virtual void arraycopy_epilogue(...) {}

  virtual void load_at(...);
  virtual void store_at(...);
  virtual void obj_equals(...);
  virtual void obj_equals(...);
  virtual void resolve(...);
  virtual void try_resolve_jobject_in_native(...);
  virtual void tlab_allocate(...);
  virtual void eden_allocate(...);
  virtual void barrier_stubs_init() {}
  virtual void nmethod_entry_barrier(...);
};
```

```cpp
void ShenandoahBarrierSetAssembler::load_at(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
             Register dst, Address src, Register tmp1, Register tmp_thread) {
  bool on_oop = type == T_OBJECT || type == T_ARRAY;
  bool in_heap = (decorators & IN_HEAP) != 0;
  bool on_weak = (decorators & ON_WEAK_OOP_REF) != 0;
  bool on_phantom = (decorators & ON_PHANTOM_OOP_REF) != 0;
  bool on_reference = on_weak || on_phantom;
  if (in_heap) {
    read_barrier_not_null(masm, src.base());
  }
  BarrierSetAssembler::load_at(masm, decorators, type, dst, src, tmp1, tmp_thread);
  if (ShenandoahKeepAliveBarrier && on_oop && on_reference) {
    const Register thread = NOT_LP64(tmp_thread) LP64_ONLY(r15_thread);
    NOT_LP64(__ get_thread(thread));

    // Generate the SATB pre-barrier code to log the value of
    // the referent field in an SATB buffer.
    shenandoah_write_barrier_pre(masm /* masm */,
                                 noreg /* obj */,
                                 dst /* pre_val */,
                                 thread /* thread */,
                                 tmp1 /* tmp */,
                                 true /* tosca_live */,
                                 true /* expand_call */);
  }
}
```
