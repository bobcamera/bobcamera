import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MaskCreationComponent } from './mask-creation.component';

describe('MaskCreationComponent', () => {
  let component: MaskCreationComponent;
  let fixture: ComponentFixture<MaskCreationComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ MaskCreationComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(MaskCreationComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
