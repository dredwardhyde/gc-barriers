# Shenandoah Garbage Collector's Barriers Evolution

All garbage collector's interpreter barriers must be derived from **BarrierSetAssembler** class which implements default barriers. Then custom barrier class must be installed during heap initialization using **BarrierSet::set_barrier_set** method along with barrier sets for C1 and C2 JIT compilers:
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
Barriers are invoked during bytecode interpretation, for example:

<img src="https://raw.githubusercontent.com/dredwardhyde/gc-barriers/master/barriers.png" width="800"/> 

# Barriers in Shenandoah 1.0
**Barriers** in Shenandoah 1.0 were implemented as a **Brooks forwarding pointer**.  
Here is how it is described in [The Garbage Collection Handbook: The Art of Automatic Memory Management](https://www.amazon.com/Garbage-Collection-Handbook-Management-Algorithms/dp/1420082795):  

<img src="https://raw.githubusercontent.com/dredwardhyde/gc-barriers/master/Screenshot%202020-04-22%20at%2009.08.12.png" width="900"/>  
<img src="https://raw.githubusercontent.com/dredwardhyde/gc-barriers/master/Screenshot%202020-04-24%20at%2010.08.21.png" width="900"/>  

## Read barrier  

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

## Write barrier
Consists of two parts: **Brook's write barrier** and **SATB barrier**:

```cpp
void ShenandoahBarrierSetAssembler::store_at(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
              Address dst, Register val, Register tmp1, Register tmp2) {

  bool in_heap = (decorators & IN_HEAP) != 0;
  bool as_normal = (decorators & AS_NORMAL) != 0;
  if (in_heap) {
    write_barrier(masm, dst.base()); // Brook's write barrier
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
      ...
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

# ACMP barrier  

Invoked, for example, during **if_acmp** opcode interpretation (when you compare two objects by references)

```cpp
void ShenandoahBarrierSetAssembler::obj_equals(MacroAssembler* masm, Register op1, Register op2) {
  __ cmpptr(op1, op2); // compare initial pointers
  if (ShenandoahAcmpBarrier) {
    Label done;
    __ jccb(Assembler::equal, done);
    read_barrier(masm, op1); // read first object's forwarding pointer
    read_barrier(masm, op2); // read second object's forwarding pointer
    __ cmpptr(op1, op2);     // compare them too
    __ bind(done);
  }
}
```

# CAS barrier  

This barrier is implemented as dedicated LIRA operation **LIR_OpShenandoahCompareAndSwap** as a part of **ShenandoahBarrierSetC1** barrier set:

```cpp
void ShenandoahBarrierSetAssembler::cmpxchg_oop(MacroAssembler* masm,
                                                Register res, Address addr, Register oldval, Register newval,
                                                bool exchange, Register tmp1, Register tmp2) {
  assert(ShenandoahCASBarrier, "Should only be used when CAS barrier is enabled");
  assert(oldval == rax, "must be in rax for implicit use in cmpxchg");

  Label retry, done;

  // Remember oldval for retry logic below
  if (UseCompressedOops) {
    __ movl(tmp1, oldval);
  } else {
    __ movptr(tmp1, oldval);
  }

  // Step 1. Try to CAS with given arguments. If successful, then we are done,
  // and can safely return.
  if (os::is_MP()) __ lock();
  if (UseCompressedOops) {
    __ cmpxchgl(newval, addr);
  } else {
    __ cmpxchgptr(newval, addr);
  }
  __ jcc(Assembler::equal, done, true);

  // Step 2. CAS had failed. This may be a false negative.
  //
  // The trouble comes when we compare the to-space pointer with the from-space
  // pointer to the same object. To resolve this, it will suffice to read both
  // oldval and the value from memory through the read barriers -- this will give
  // both to-space pointers. If they mismatch, then it was a legitimate failure.
  //
  if (UseCompressedOops) {
    __ decode_heap_oop(tmp1);
  }
  read_barrier_impl(masm, tmp1);

  if (UseCompressedOops) {
    __ movl(tmp2, oldval);
    __ decode_heap_oop(tmp2);
  } else {
    __ movptr(tmp2, oldval);
  }
  read_barrier_impl(masm, tmp2);

  __ cmpptr(tmp1, tmp2);
  __ jcc(Assembler::notEqual, done, true);

  // Step 3. Try to CAS again with resolved to-space pointers.
  //
  // Corner case: it may happen that somebody stored the from-space pointer
  // to memory while we were preparing for retry. Therefore, we can fail again
  // on retry, and so need to do this in loop, always re-reading the failure
  // witness through the read barrier.
  __ bind(retry);
  if (os::is_MP()) __ lock();
  if (UseCompressedOops) {
    __ cmpxchgl(newval, addr);
  } else {
    __ cmpxchgptr(newval, addr);
  }
  __ jcc(Assembler::equal, done, true);

  if (UseCompressedOops) {
    __ movl(tmp2, oldval);
    __ decode_heap_oop(tmp2);
  } else {
    __ movptr(tmp2, oldval);
  }
  read_barrier_impl(masm, tmp2);

  __ cmpptr(tmp1, tmp2);
  __ jcc(Assembler::equal, retry, true);

  // Step 4. If we need a boolean result out of CAS, check the flag again,
  // and promote the result. Note that we handle the flag from both the CAS
  // itself and from the retry loop.
  __ bind(done);
  if (!exchange) {
    assert(res != NULL, "need result register");
    __ setb(Assembler::equal, res);
    __ movzbl(res, res);
  }
}
```

# Barriers in Shenandoah 2.0+  

In version 2.0 barriers were implemented as Load Reference Barriers - objects could be evacuated during its reference resolution, writes and reads always happen into/from the to-space copy (strong invariant).

Key changes:

You could read more about Shenandoah 2.0+ architecture in the following articles by [Roman Kennke](https://twitter.com/rkennke):  
[Shenandoah GC in JDK 13, Part 1: Load reference barriers](https://developers.redhat.com/blog/2019/06/27/shenandoah-gc-in-jdk-13-part-1-load-reference-barriers/)  
[Shenandoah GC in JDK 13, Part 2: Eliminating the forward pointer word](https://developers.redhat.com/blog/2019/06/28/shenandoah-gc-in-jdk-13-part-2-eliminating-the-forward-pointer-word/)  
[Shenandoah GC in JDK 14, Part 1: Self-fixing barriers](https://developers.redhat.com/blog/2020/03/04/shenandoah-gc-in-jdk-14-part-1-self-fixing-barriers/)  

In essence, Shenandoah designers took similar approach as Azul in their [C4 Garbage Collector](https://www.azul.com/resources/azul-technology/azul-c4-garbage-collector/).  
Excerpt from  [Azul C4 white paper](http://go.azul.com/continuously-concurrent-compacting-collector):  
<img src="https://raw.githubusercontent.com/dredwardhyde/gc-barriers/master/lvb_azul.png" width="500"/>  
  
## Write barrier

As you can see, only **SATB barrier** is still here, no **Brook's barrier**:  

```cpp
void ShenandoahBarrierSetAssembler::store_at(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
              Address dst, Register val, Register tmp1, Register tmp2) {

  bool on_oop = is_reference_type(type);
  bool in_heap = (decorators & IN_HEAP) != 0;
  bool as_normal = (decorators & AS_NORMAL) != 0;
  if (on_oop && in_heap) {
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

    assert_different_registers(val, tmp1, tmp2, tmp3, rthread);
    
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
    NOT_LP64(imasm->restore_bcp());
  } else {
    BarrierSetAssembler::store_at(masm, decorators, type, dst, val, tmp1, tmp2);
  }
}
```

## Read barrier  

```cpp
void ShenandoahBarrierSetAssembler::load_at(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
             Register dst, Address src, Register tmp1, Register tmp_thread) {
  // 1: non-reference load, no additional barrier is needed
  if (!is_reference_type(type)) {
    BarrierSetAssembler::load_at(masm, decorators, type, dst, src, tmp1, tmp_thread);
    return;
  }

  assert((decorators & ON_UNKNOWN_OOP_REF) == 0, "Not expected");

  // 2: load a reference from src location and apply LRB if needed
  if (ShenandoahBarrierSet::need_load_reference_barrier(decorators, type)) {
    Register result_dst = dst;
    bool use_tmp1_for_dst = false;

    // Preserve src location for LRB
    if (dst == src.base() || dst == src.index()) {
      // Use tmp1 for dst if possible, as it is not used in BarrierAssembler::load_at()
      if (tmp1->is_valid() && tmp1 != src.base() && tmp1 != src.index()) {
        dst = tmp1;
        use_tmp1_for_dst = true;
      } else {
        dst = rdi;
        __ push(dst);
      }
      assert_different_registers(dst, src.base(), src.index());
    }

    BarrierSetAssembler::load_at(masm, decorators, type, dst, src, tmp1, tmp_thread);

    if (ShenandoahBarrierSet::use_load_reference_barrier_native(decorators, type)) {
      load_reference_barrier_native(masm, dst, src);
    } else {
      load_reference_barrier(masm, dst, src);
    }

    // Move loaded oop to final destination
    if (dst != result_dst) {
      __ movptr(result_dst, dst);

      if (!use_tmp1_for_dst) {
        __ pop(dst);
      }

      dst = result_dst;
    }
  } else {
    BarrierSetAssembler::load_at(masm, decorators, type, dst, src, tmp1, tmp_thread);
  }

  // 3: apply keep-alive barrier if needed
  if (ShenandoahBarrierSet::need_keep_alive_barrier(decorators, type)) {
    __ push_IU_state();
    // That path can be reached from the c2i adapter with live fp
    // arguments in registers.
    LP64_ONLY(assert(Argument::n_float_register_parameters_j == 8, "8 fp registers to save at java call"));

    ...

    Register thread = NOT_LP64(tmp_thread) LP64_ONLY(r15_thread);
    assert_different_registers(dst, tmp1, tmp_thread);
    if (!thread->is_valid()) {
      thread = rdx;
    }
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
    ...
  }
}
```
