import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MaskControlsComponent } from './mask-controls.component';

describe('MaskControlsComponent', () => {
  let component: MaskControlsComponent;
  let fixture: ComponentFixture<MaskControlsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ MaskControlsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(MaskControlsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
