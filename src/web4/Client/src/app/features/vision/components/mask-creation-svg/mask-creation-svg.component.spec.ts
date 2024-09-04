import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MaskCreationSvgComponent } from './mask-creation-svg.component';

describe('MaskCreationSvgComponent', () => {
  let component: MaskCreationSvgComponent;
  let fixture: ComponentFixture<MaskCreationSvgComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ MaskCreationSvgComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(MaskCreationSvgComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
