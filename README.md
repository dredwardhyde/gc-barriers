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
  
...

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

# Barriers in JDK <= 13
**Barriers** in Shenandoah is JDK versions before 13 implemented as a **Brooks forwarding pointer**.  
Here is how it is described in [The Garbage Collection Handbook: The Art of Automatic Memory Management](https://www.amazon.com/Garbage-Collection-Handbook-Management-Algorithms/dp/1420082795):  

<img src="https://raw.githubusercontent.com/dredwardhyde/gc-barriers/master/Screenshot%202020-04-22%20at%2009.08.12.png" width="900"/>  
<img src="https://raw.githubusercontent.com/dredwardhyde/gc-barriers/master/Screenshot%202020-04-24%20at%2010.08.21.png" width="900"/>  

**Read barrier**:  

```cpp
void ShenandoahBarrierSetAssembler::load_at(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
             Register dst, Address src, Register tmp1, Register tmp_thread) {
  bool on_oop = type == T_OBJECT || type == T_ARRAY;
  bool in_heap = (decorators & IN_HEAP) != 0;
  bool on_weak = (decorators & ON_WEAK_OOP_REF) != 0;
  bool on_phantom = (decorators & ON_PHANTOM_OOP_REF) != 0;
  bool on_reference = on_weak || on_phantom;
  if (in_heap) {
    read_barrier_not_null(masm, src.base()); // read barrier
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

void ShenandoahBarrierSetAssembler::read_barrier_not_null(MacroAssembler* masm, Register dst) {
  if (ShenandoahReadBarrier) {
    read_barrier_not_null_impl(masm, dst);
  }
}

void ShenandoahBarrierSetAssembler::read_barrier_not_null_impl(MacroAssembler* masm, Register dst) {
  assert(UseShenandoahGC && (ShenandoahReadBarrier || ShenandoahStoreValReadBarrier || ShenandoahCASBarrier), "should be enabled");
  __ movptr(dst, Address(dst, ShenandoahBrooksPointer::byte_offset())); // looking for forwarding pointer
}
```
Forwarding pointer is located in memory exactly before target object, so it could be resolved using only one instruction:
```cpp
/* Offset from the object start, in bytes. */
static inline int byte_offset() {
  return -HeapWordSize; // exactly one HeapWord
}
```

**Write barrier** consists of two parts: **'main' barrier** and **SATB barrier**:

```cpp
void ShenandoahBarrierSetAssembler::store_at(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
              Address dst, Register val, Register tmp1, Register tmp2) {

  bool in_heap = (decorators & IN_HEAP) != 0;
  bool as_normal = (decorators & AS_NORMAL) != 0;
  if (in_heap) {
    write_barrier(masm, dst.base()); // 'Main' barrier
  }
  // Code below belongs to SATB barrier
  if (type == T_OBJECT || type == T_ARRAY) {
    bool needs_pre_barrier = as_normal;

    Register tmp3 = LP64_ONLY(r8) NOT_LP64(rsi);
    Register rthread = LP64_ONLY(r15_thread) NOT_LP64(rcx);
    // flatten object address if needed
    // We do it regardless of precise because we need the registers
    if (dst.index() == noreg && dst.disp() == 0) {
      if (dst.base() != tmp1) {
        __ movptr(tmp1, dst.base());
      }
    } else {
      __ lea(tmp1, dst);
    }

    if (needs_pre_barrier) {
      shenandoah_write_barrier_pre(masm /*masm*/,
                                   tmp1 /* obj */,
                                   tmp2 /* pre_val */,
                                   rthread /* thread */,
                                   tmp3  /* tmp */,
                                   val != noreg /* tosca_live */,
                                   false /* expand_call */);
    }
    if (val == noreg) {
      BarrierSetAssembler::store_at(masm, decorators, type, Address(tmp1, 0), val, noreg, noreg);
    } else {
      storeval_barrier(masm, val, tmp3);
      BarrierSetAssembler::store_at(masm, decorators, type, Address(tmp1, 0), val, noreg, noreg);
    }
  } else {
    BarrierSetAssembler::store_at(masm, decorators, type, dst, val, tmp1, tmp2);
  }
}
```

```cpp
void ShenandoahBarrierSetAssembler::write_barrier(MacroAssembler* masm, Register dst) {
  if (ShenandoahWriteBarrier) {
    write_barrier_impl(masm, dst);
  }
}

void ShenandoahBarrierSetAssembler::write_barrier_impl(MacroAssembler* masm, Register dst) {
  assert(UseShenandoahGC && (ShenandoahWriteBarrier || ShenandoahStoreValEnqueueBarrier), "Should be enabled");
  Label done;

  Address gc_state(r15_thread, in_bytes(ShenandoahThreadLocalData::gc_state_offset()));
  __ testb(gc_state, ShenandoahHeap::HAS_FORWARDED | ShenandoahHeap::EVACUATION | ShenandoahHeap::TRAVERSAL);
  __ jccb(Assembler::zero, done);

  // Heap is unstable, need to perform the read-barrier even if WB is inactive
  read_barrier_not_null(masm, dst);

  __ testb(gc_state, ShenandoahHeap::EVACUATION | ShenandoahHeap::TRAVERSAL);
  __ jccb(Assembler::zero, done);

   if (dst != rax) {
     __ xchgptr(dst, rax); // Move obj into rax and save rax into obj.
   }

   __ call(RuntimeAddress(CAST_FROM_FN_PTR(address, ShenandoahBarrierSetAssembler::shenandoah_wb())));

   if (dst != rax) {
     __ xchgptr(rax, dst); // Swap back obj with rax.
   }

  __ bind(done);
}
```
Check if object is in the collection set and prepare slow path (object evacuation):
```cpp
address ShenandoahBarrierSetAssembler::generate_shenandoah_wb(StubCodeGenerator* cgen) {
  __ align(CodeEntryAlignment);
  StubCodeMark mark(cgen, "StubRoutines", "shenandoah_wb");
  address start = __ pc();

  Label not_done;
  __ push(rdi);
  __ push(r8);

  // Check for object beeing in the collection set.
  __ mov(rdi, rax);
  __ shrptr(rdi, ShenandoahHeapRegion::region_size_bytes_shift_jint());
  __ movptr(r8, (intptr_t) ShenandoahHeap::in_cset_fast_test_addr());
  __ movbool(r8, Address(r8, rdi, Address::times_1));
  __ testbool(r8); // live or not
  __ jccb(Assembler::notZero, not_done);

  __ pop(r8);
  __ pop(rdi);
  __ ret(0);

  __ bind(not_done);

     ...
     
  save_vector_registers(cgen->assembler());
  __ movptr(rdi, rax);
  __ call_VM_leaf(CAST_FROM_FN_PTR(address, ShenandoahRuntime::write_barrier_JRT), rdi);
  restore_vector_registers(cgen->assembler());
  
     ...

  __ ret(0);
  return start;
}
```
Check if object is already evacuated using **oopDesc::equals_raw(obj, fwd)**, then try to evacuate and return new forward pointer:
```cpp
JRT_LEAF(oopDesc*, ShenandoahRuntime::write_barrier_JRT(oopDesc* src))
  oop result = ShenandoahBarrierSet::barrier_set()->write_barrier_mutator(src);
  return (oopDesc*) result;
JRT_END
```

```cpp
oop ShenandoahBarrierSet::write_barrier_mutator(oop obj) {
  assert(UseShenandoahGC && ShenandoahWriteBarrier, "should be enabled");
  assert(_heap->is_gc_in_progress_mask(ShenandoahHeap::EVACUATION | ShenandoahHeap::TRAVERSAL), "evac should be in progress");
  shenandoah_assert_in_cset(NULL, obj);

  oop fwd = resolve_forwarded_not_null(obj);
  if (oopDesc::equals_raw(obj, fwd)) {
    ShenandoahEvacOOMScope oom_evac_scope;

    Thread* thread = Thread::current();
    oop res_oop = _heap->evacuate_object(obj, thread);

    // Since we are already here and paid the price of getting through runtime call adapters
    // and acquiring oom-scope, it makes sense to try and evacuate more adjacent objects,
    // thus amortizing the overhead. For sparsely live heaps, scan costs easily dominate
    // total assist costs, and can introduce a lot of evacuation latency. This is why we
    // only scan for _nearest_ N objects, regardless if they are eligible for evac or not.
    // The scan itself should also avoid touching the non-marked objects below TAMS, because
    // their metadata (notably, klasses) may be incorrect already.

    size_t max = ShenandoahEvacAssist;
    if (max > 0) {
      // Try to evacuate up to 'max' adjacent objects
    }

    return res_oop;
  }
  return fwd;
}
```
Object evacuation occurs here. Note that Shenandoah GC never evacuate **humongous objects** like **huge arrays of primitives**.
At first, object size is determined, then multiple memory allocation attempts occur. At first, it attempts to allocate memory from **GCLab** related to current thread. If it fails, then try to allocate from **shared GC memory**. If this attempt also fails, it means **OOM** occurred during allocation.

After successful allocation, **copy object and try to install new forwarding pointer**. After successful pointer installation, consider this new copy as public. Otherwise, it is important to undo GCLab allocation or overwrite shared GC memory allocation:
```cpp
inline oop ShenandoahHeap::evacuate_object(oop p, Thread* thread) {
  if (ShenandoahThreadLocalData::is_oom_during_evac(Thread::current())) {
    // This thread went through the OOM during evac protocol and it is safe to return
    // the forward pointer. It must not attempt to evacuate any more.
    return ShenandoahBarrierSet::resolve_forwarded(p);
  }

  assert(ShenandoahThreadLocalData::is_evac_allowed(thread), "must be enclosed in oom-evac scope");

  size_t size_no_fwdptr = (size_t) p->size();
  size_t size_with_fwdptr = size_no_fwdptr + ShenandoahBrooksPointer::word_size();

  assert(!heap_region_containing(p)->is_humongous(), "never evacuate humongous objects");

  bool alloc_from_gclab = true;
  HeapWord* filler = NULL;

  if (UseTLAB) {
    filler = allocate_from_gclab(thread, size_with_fwdptr);
  }
  if (filler == NULL) {
    ShenandoahAllocRequest req = ShenandoahAllocRequest::for_shared_gc(size_with_fwdptr);
    filler = allocate_memory(req);
    alloc_from_gclab = false;
  }

  if (filler == NULL) {
    control_thread()->handle_alloc_failure_evac(size_with_fwdptr);

    _oom_evac_handler.handle_out_of_memory_during_evacuation();

    return ShenandoahBarrierSet::resolve_forwarded(p);
  }

  // Copy the object and initialize its forwarding ptr:
  HeapWord* copy = filler + ShenandoahBrooksPointer::word_size();
  oop copy_val = oop(copy);

  Copy::aligned_disjoint_words((HeapWord*) p, copy, size_no_fwdptr);
  ShenandoahBrooksPointer::initialize(oop(copy));

  // Try to install the new forwarding pointer.
  oop result = ShenandoahBrooksPointer::try_update_forwardee(p, copy_val);

  if (oopDesc::equals_raw(result, p)) {
    // Successfully evacuated. Our copy is now the public one!
    shenandoah_assert_correct(NULL, copy_val);
    return copy_val;
  }  else {
    // Failed to evacuate. We need to deal with the object that is left behind. Since this
    // new allocation is certainly after TAMS, it will be considered live in the next cycle.
    // But if it happens to contain references to evacuated regions, those references would
    // not get updated for this stale copy during this cycle, and we will crash while scanning
    // it the next cycle.
    //
    // For GCLAB allocations, it is enough to rollback the allocation ptr. Either the next
    // object will overwrite this stale copy, or the filler object on LAB retirement will
    // do this. For non-GCLAB allocations, we have no way to retract the allocation, and
    // have to explicitly overwrite the copy with the filler object. With that overwrite,
    // we have to keep the fwdptr initialized and pointing to our (stale) copy.
    if (alloc_from_gclab) {
      ShenandoahThreadLocalData::gclab(thread)->undo_allocation(filler, size_with_fwdptr);
    } else {
      fill_with_object(copy, size_no_fwdptr);
    }
    shenandoah_assert_correct(NULL, copy_val);
    shenandoah_assert_correct(NULL, result);
    return result;
  }
}
```
