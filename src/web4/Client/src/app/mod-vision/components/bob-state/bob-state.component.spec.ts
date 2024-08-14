import { ComponentFixture, TestBed } from '@angular/core/testing';

import { BobStateComponent } from './bob-state.component';

describe('BobStateComponent', () => {
  let component: BobStateComponent;
  let fixture: ComponentFixture<BobStateComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ BobStateComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(BobStateComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
